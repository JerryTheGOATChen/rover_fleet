"""MAPPO training: one actor shared by every robot, a critic that sees the true state.

    ~/venvs/fleet/bin/python -m fleet_rl.train --name main --updates 2000

Every episode draws a random house, fleet size (2-4), radio range (3-12 m),
beacon count (0-2), robot speeds and maybe a robot failure, so one model
covers every setting. Environments run in parallel processes; the main
process batches all robots of all environments through the networks.

Writes runs/<name>/progress.csv, checkpoints, and actor weights (.npz) that
policy.LearnedPolicy loads.
"""

import argparse
import csv
import os
import time

import numpy as np
import torch
import torch.multiprocessing as mp
from torch.distributions import Categorical

from .config import MAX_ROBOTS, sample_training_config
from .houses import generate_house
from .model import Actor, Critic, export_actor
from .observations import MAX_OPTIONS, OPTION_DIM, SELF_DIM, SLOT_STAY
from .sim import FleetSim

TRAIN_SEEDS = 100_000   # houses 0..99,999; evaluation uses seeds >= 1,000,000


def pack(sim, observations):
    """Fixed-shape arrays for one environment (robot slots beyond the fleet are padding)."""
    options = np.zeros((MAX_ROBOTS, MAX_OPTIONS, OPTION_DIM), np.float32)
    mask = np.zeros((MAX_ROBOTS, MAX_OPTIONS), bool)
    mask[:, SLOT_STAY] = True                   # padding rows: a valid no-op
    selff = np.zeros((MAX_ROBOTS, SELF_DIM), np.float32)
    acting = np.zeros(MAX_ROBOTS, bool)
    for i, obs in enumerate(observations):
        options[i], mask[i], selff[i] = obs.options, obs.mask, obs.self_features
        acting[i] = sim.robots[i].alive
    robots, present, scalars = sim.global_state()
    return dict(options=options, mask=mask, selff=selff, acting=acting,
                robots=robots, present=present, scalars=scalars)


def worker(conn, seed):
    torch.set_num_threads(1)
    rng = np.random.default_rng(seed)

    def new_episode():
        cfg = sample_training_config(rng)
        house = generate_house(int(rng.integers(TRAIN_SEEDS)), sim=cfg)
        return FleetSim(house, cfg, seed=int(rng.integers(2**31)))

    sim, ep_return = new_episode(), 0.0
    conn.send(pack(sim, sim.observe()))
    while True:
        cmd, actions = conn.recv()
        if cmd == 'close':
            return
        reward = sim.step(actions[:sim.cfg.n_robots])
        ep_return += reward
        done, info = sim.done, None
        if done:
            info = dict(ret=ep_return, final=sim.base_coverage,
                        auc=float(np.mean(sim.history['base'])),
                        n=sim.cfg.n_robots, comm=sim.cfg.comm_range)
            sim, ep_return = new_episode(), 0.0
        conn.send((pack(sim, sim.observe()), reward, done, info))


def stack(batch):
    return {k: torch.as_tensor(np.stack([b[k] for b in batch])) for k in batch[0]}


def actor_logits(actor, obs):
    """Run every robot of every environment through the actor: (E, R, O) logits."""
    e = obs['options'].shape[0]
    logits = actor(obs['options'].view(e * MAX_ROBOTS, MAX_OPTIONS, OPTION_DIM),
                   obs['mask'].view(e * MAX_ROBOTS, MAX_OPTIONS),
                   obs['selff'].view(e * MAX_ROBOTS, SELF_DIM))
    return logits.view(e, MAX_ROBOTS, MAX_OPTIONS)


def train(args):
    torch.manual_seed(args.seed)
    torch.set_num_threads(args.torch_threads)
    out = os.path.join(args.runs, args.name)
    os.makedirs(out, exist_ok=True)

    ctx = mp.get_context('spawn')   # fork + torch threads can deadlock
    pipes, procs = [], []
    for i in range(args.envs):
        parent, child = ctx.Pipe()
        proc = ctx.Process(target=worker, args=(child, args.seed * 1000 + i), daemon=True)
        proc.start()
        pipes.append(parent)
        procs.append(proc)
    obs = [p.recv() for p in pipes]

    actor, critic = Actor(), Critic()
    if args.resume:
        state = torch.load(args.resume)
        actor.load_state_dict(state['actor'])
        critic.load_state_dict(state['critic'])
    actor_opt = torch.optim.Adam(actor.parameters(), lr=args.lr, eps=1e-5)
    critic_opt = torch.optim.Adam(critic.parameters(), lr=args.lr, eps=1e-5)

    log = open(os.path.join(out, 'progress.csv'), 'w', newline='')
    fields = ['update', 'env_steps', 'seconds', 'episodes', 'return', 'final_coverage',
              'mean_coverage', 'policy_loss', 'value_loss', 'entropy', 'clip_frac', 'lr']
    writer = csv.DictWriter(log, fieldnames=fields)
    writer.writeheader()

    t_start, env_steps = time.time(), 0
    T, E = args.steps, args.envs
    for update in range(1, args.updates + 1):
        lr = args.lr * (1.0 - (update - 1) / args.updates)
        for opt in (actor_opt, critic_opt):
            for group in opt.param_groups:
                group['lr'] = lr

        # ---- collect T decisions in each of E environments
        traj = {k: [] for k in ('obs', 'actions', 'logp', 'values', 'rewards', 'dones')}
        episodes = []
        for _ in range(T):
            o = stack(obs)
            with torch.no_grad():
                dist = Categorical(logits=actor_logits(actor, o))
                actions = dist.sample()
                logp = dist.log_prob(actions)
                values = critic(o['robots'], o['present'], o['scalars'])
            for pipe, a in zip(pipes, actions.numpy()):
                pipe.send(('step', a))
            results = [p.recv() for p in pipes]
            obs = [r[0] for r in results]
            episodes += [r[3] for r in results if r[3] is not None]
            traj['obs'].append(o)
            traj['actions'].append(actions)
            traj['logp'].append(logp)
            traj['values'].append(values)
            traj['rewards'].append(torch.tensor([r[1] for r in results], dtype=torch.float32))
            traj['dones'].append(torch.tensor([r[2] for r in results], dtype=torch.float32))
        env_steps += T * E

        # ---- advantages (GAE); the team shares one reward and one value
        with torch.no_grad():
            o = stack(obs)
            next_value = critic(o['robots'], o['present'], o['scalars'])
        values = torch.stack(traj['values'])
        rewards, dones = torch.stack(traj['rewards']), torch.stack(traj['dones'])
        adv = torch.zeros_like(rewards)
        running = torch.zeros(E)
        for t in reversed(range(T)):
            nxt = next_value if t == T - 1 else values[t + 1]
            alive = 1.0 - dones[t]
            delta = rewards[t] + args.gamma * nxt * alive - values[t]
            running = delta + args.gamma * args.lam * alive * running
            adv[t] = running
        returns = adv + values

        flat = {k: torch.cat([o[k] for o in traj['obs']]) for k in traj['obs'][0]}
        flat_actions = torch.cat(traj['actions'])
        flat_logp = torch.cat(traj['logp'])
        flat_adv = adv.reshape(-1)
        flat_adv = (flat_adv - flat_adv.mean()) / (flat_adv.std() + 1e-8)
        flat_ret = returns.reshape(-1)

        # ---- PPO update
        n = T * E
        stats = {'policy_loss': [], 'value_loss': [], 'entropy': [], 'clip_frac': []}
        for _ in range(args.epochs):
            perm = torch.randperm(n)
            for mb in perm.chunk(args.minibatches):
                o = {k: v[mb] for k, v in flat.items()}
                acting = o['acting'].float()
                dist = Categorical(logits=actor_logits(actor, o))
                logp = dist.log_prob(flat_actions[mb])
                ratio = torch.exp(logp - flat_logp[mb])
                a = flat_adv[mb][:, None]
                surrogate = torch.min(ratio * a, ratio.clamp(1 - args.clip, 1 + args.clip) * a)
                denom = acting.sum().clamp(min=1.0)
                policy_loss = -(surrogate * acting).sum() / denom
                entropy = (dist.entropy() * acting).sum() / denom
                actor_opt.zero_grad()
                (policy_loss - args.ent_coef * entropy).backward()
                torch.nn.utils.clip_grad_norm_(actor.parameters(), args.max_grad_norm)
                actor_opt.step()

                value_loss = 0.5 * (critic(o['robots'], o['present'], o['scalars'])
                                    - flat_ret[mb]).pow(2).mean()
                critic_opt.zero_grad()
                value_loss.backward()
                torch.nn.utils.clip_grad_norm_(critic.parameters(), args.max_grad_norm)
                critic_opt.step()

                stats['policy_loss'].append(policy_loss.item())
                stats['value_loss'].append(value_loss.item())
                stats['entropy'].append(entropy.item())
                clipped = ((ratio - 1).abs() > args.clip).float()
                stats['clip_frac'].append(((clipped * acting).sum() / denom).item())

        row = dict(update=update, env_steps=env_steps, seconds=round(time.time() - t_start, 1),
                   episodes=len(episodes), lr=lr,
                   **{k: float(np.mean(v)) for k, v in stats.items()})
        if episodes:
            row['return'] = float(np.mean([e['ret'] for e in episodes]))
            row['final_coverage'] = float(np.mean([e['final'] for e in episodes]))
            row['mean_coverage'] = float(np.mean([e['auc'] for e in episodes]))
        writer.writerow(row)
        log.flush()
        if update % args.log_every == 0 or update == 1:
            print(f"update {update:5d}  steps {env_steps:8d}  {row['seconds']:7.0f}s  "
                  f"return {row.get('return', float('nan')):6.2f}  "
                  f"mean cov {row.get('mean_coverage', float('nan')):.3f}  "
                  f"ent {row['entropy']:.2f}  vloss {row['value_loss']:.3f}", flush=True)
        if update % args.save_every == 0 or update == args.updates:
            torch.save({'actor': actor.state_dict(), 'critic': critic.state_dict(),
                        'update': update}, os.path.join(out, 'checkpoint.pt'))
            export_actor(actor, os.path.join(out, f'actor_{update:05d}.npz'))
            export_actor(actor, os.path.join(out, 'actor_latest.npz'))

    for pipe in pipes:
        pipe.send(('close', None))
    for proc in procs:
        proc.join(timeout=5)
    log.close()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument('--name', default='main')
    p.add_argument('--runs', default='runs')
    p.add_argument('--updates', type=int, default=2000)
    p.add_argument('--envs', type=int, default=8)
    p.add_argument('--steps', type=int, default=80, help='decisions per env per update')
    p.add_argument('--lr', type=float, default=3e-4)
    p.add_argument('--gamma', type=float, default=0.99)
    p.add_argument('--lam', type=float, default=0.95)
    p.add_argument('--clip', type=float, default=0.2)
    p.add_argument('--ent-coef', type=float, default=0.01)
    p.add_argument('--epochs', type=int, default=4)
    p.add_argument('--minibatches', type=int, default=4)
    p.add_argument('--max-grad-norm', type=float, default=0.5)
    p.add_argument('--seed', type=int, default=1)
    p.add_argument('--torch-threads', type=int, default=2)
    p.add_argument('--log-every', type=int, default=10)
    p.add_argument('--save-every', type=int, default=100)
    p.add_argument('--resume', default='')
    train(p.parse_args())


if __name__ == '__main__':
    main()

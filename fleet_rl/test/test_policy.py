"""The numpy policy matches the torch actor, the shipped model works, training runs."""

import os

import numpy as np
import pytest

from fleet_rl.config import SimConfig
from fleet_rl.houses import generate_house
from fleet_rl.observations import MAX_OPTIONS, OPTION_DIM, SELF_DIM
from fleet_rl.policy import DEFAULT_MODEL, LearnedPolicy
from fleet_rl.sim import FleetSim


def test_numpy_policy_matches_torch(tmp_path):
    torch = pytest.importorskip('torch')
    from fleet_rl.model import Actor, export_actor

    torch.manual_seed(0)
    actor = Actor()
    for p in actor.parameters():           # non-trivial weights everywhere
        torch.nn.init.normal_(p, std=0.3)
    export_actor(actor, tmp_path / 'a.npz')
    policy = LearnedPolicy(str(tmp_path / 'a.npz'))

    rng = np.random.default_rng(0)
    for _ in range(20):
        options = rng.normal(size=(MAX_OPTIONS, OPTION_DIM)).astype(np.float32)
        selff = rng.normal(size=SELF_DIM).astype(np.float32)
        mask = rng.random(MAX_OPTIONS) < 0.6
        mask[0] = True
        with torch.no_grad():
            expected = actor(torch.tensor(options)[None], torch.tensor(mask)[None],
                             torch.tensor(selff)[None])[0].numpy()
        got = policy.logits(options, mask, selff)
        np.testing.assert_allclose(got[mask], expected[mask], rtol=1e-4, atol=1e-4)
        assert (got[~mask] < -1e8).all()


@pytest.mark.skipif(not os.path.exists(DEFAULT_MODEL), reason='no trained model yet')
def test_shipped_model_picks_valid_options():
    sim = FleetSim(generate_house(1_000_001), SimConfig(beacons_per_robot=1), seed=0)
    policy = LearnedPolicy()
    while not sim.done:
        observations = sim.observe()
        actions = [policy.act(o) for o in observations]
        assert all(o.mask[a] for o, a in zip(observations, actions))
        sim.step(actions)


def test_training_smoke(tmp_path):
    pytest.importorskip('torch')
    from argparse import Namespace

    from fleet_rl.train import train

    args = Namespace(name='smoke', runs=str(tmp_path), updates=2, envs=2, steps=4, lr=3e-4,
                     gamma=0.99, lam=0.95, clip=0.2, ent_coef=0.01, epochs=2, minibatches=2,
                     max_grad_norm=0.5, seed=0, torch_threads=1, log_every=1, save_every=1,
                     resume='')
    train(args)
    out = tmp_path / 'smoke'
    assert (out / 'actor_latest.npz').exists() and (out / 'checkpoint.pt').exists()
    LearnedPolicy(str(out / 'actor_latest.npz'))
    assert len((out / 'progress.csv').read_text().splitlines()) == 3

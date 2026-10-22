"""The networks (PyTorch, training only).

Actor: scores each of a robot's options. Every option and the robot's own
situation become 64-d tokens; one attention layer lets each option "look at"
the others (is a teammate already near this frontier? is the base closer?);
a small head turns each option token into a score. Softmax over the valid
options gives the policy. The same weights serve every robot.

Critic: estimates how well the team will do from the true global state.
It is only used to train the actor (centralized training, decentralized
execution), so it may see things robots can't.

policy.py runs the actor in plain numpy for deployment; test_policy.py
checks the two agree.
"""

import math

import numpy as np
import torch
from torch import nn

from .observations import OPTION_DIM, SELF_DIM
from .sim import GLOBAL_DIM, ROBOT_STATE_DIM

D_MODEL, HEADS, FFN = 64, 4, 128
NEG = -1e9   # "minus infinity" for masked options; finite so softmax never sees NaN


def mlp(n_in, n_hidden, n_out):
    return nn.Sequential(nn.Linear(n_in, n_hidden), nn.ReLU(), nn.Linear(n_hidden, n_out))


class Actor(nn.Module):
    def __init__(self):
        super().__init__()
        self.opt_enc = mlp(OPTION_DIM, D_MODEL, D_MODEL)
        self.self_enc = mlp(SELF_DIM, D_MODEL, D_MODEL)
        self.ln1 = nn.LayerNorm(D_MODEL)
        self.q = nn.Linear(D_MODEL, D_MODEL)
        self.k = nn.Linear(D_MODEL, D_MODEL)
        self.v = nn.Linear(D_MODEL, D_MODEL)
        self.o = nn.Linear(D_MODEL, D_MODEL)
        self.ln2 = nn.LayerNorm(D_MODEL)
        self.ff1 = nn.Linear(D_MODEL, FFN)
        self.ff2 = nn.Linear(FFN, D_MODEL)
        self.head = mlp(D_MODEL, D_MODEL, 1)
        with torch.no_grad():   # start close to uniform over valid options
            self.head[2].weight.mul_(0.01)
            self.head[2].bias.zero_()

    def forward(self, options, mask, selff):
        """options (B, O, OPTION_DIM), mask (B, O) bool, selff (B, SELF_DIM) -> logits (B, O)."""
        b, o, _ = options.shape
        x = torch.cat([self.self_enc(selff)[:, None], self.opt_enc(options)], dim=1)
        keys_ok = torch.cat([torch.ones(b, 1, dtype=torch.bool, device=mask.device), mask], dim=1)

        h = self.ln1(x)
        dh = D_MODEL // HEADS

        def heads(t):
            return t.view(b, o + 1, HEADS, dh).transpose(1, 2)    # (B, H, T, dh)

        q, k, v = heads(self.q(h)), heads(self.k(h)), heads(self.v(h))
        scores = q @ k.transpose(-1, -2) / math.sqrt(dh)
        scores = scores.masked_fill(~keys_ok[:, None, None, :], NEG)
        att = torch.softmax(scores, dim=-1) @ v                   # (B, H, T, dh)
        x = x + self.o(att.transpose(1, 2).reshape(b, o + 1, D_MODEL))
        x = x + self.ff2(torch.relu(self.ff1(self.ln2(x))))

        logits = self.head(x[:, 1:]).squeeze(-1)
        return logits.masked_fill(~mask, NEG)


class Critic(nn.Module):
    def __init__(self):
        super().__init__()
        self.robot_enc = mlp(ROBOT_STATE_DIM, D_MODEL, D_MODEL)
        self.out = nn.Sequential(nn.Linear(2 * D_MODEL + GLOBAL_DIM, 128), nn.ReLU(),
                                 nn.Linear(128, 128), nn.ReLU(), nn.Linear(128, 1))

    def forward(self, robots, present, scalars):
        """robots (B, R, ROBOT_STATE_DIM), present (B, R) bool, scalars (B, GLOBAL_DIM) -> (B,)."""
        e = self.robot_enc(robots)
        m = present[..., None].float()
        mean = (e * m).sum(1) / m.sum(1).clamp(min=1.0)
        top = e.masked_fill(~present[..., None], NEG).max(1).values
        return self.out(torch.cat([mean, top, scalars], dim=-1)).squeeze(-1)


def export_actor(actor: Actor, path):
    """Save the actor's weights for policy.LearnedPolicy (numpy)."""
    np.savez(path, **{k: v.detach().cpu().numpy() for k, v in actor.state_dict().items()})

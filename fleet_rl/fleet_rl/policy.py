"""The trained policy, run with numpy only (no PyTorch needed at deployment).

Mirrors model.Actor exactly; test_policy.py compares the two on random inputs.
"""

import os

import numpy as np

D_MODEL, HEADS = 64, 4
NEG = -1e9
DEFAULT_MODEL = os.path.join(os.path.dirname(__file__), 'models', 'policy.npz')


def _relu(x):
    return np.maximum(x, 0.0)


class LearnedPolicy:
    name = 'learned'

    def __init__(self, path: str = DEFAULT_MODEL, sample: bool = False, seed: int = 0):
        with np.load(path) as data:
            self.w = {k: data[k].astype(np.float64) for k in data.files}
        self.sample = sample
        self.rng = np.random.default_rng(seed)

    def _linear(self, x, name):
        return x @ self.w[name + '.weight'].T + self.w[name + '.bias']

    def _mlp(self, x, name):
        return self._linear(_relu(self._linear(x, name + '.0')), name + '.2')

    def _layer_norm(self, x, name, eps=1e-5):
        mean = x.mean(-1, keepdims=True)
        var = x.var(-1, keepdims=True)
        return (x - mean) / np.sqrt(var + eps) * self.w[name + '.weight'] + self.w[name + '.bias']

    def logits(self, options, mask, selff):
        """Scores for one robot's options (invalid ones get NEG)."""
        x = np.concatenate([self._mlp(selff[None], 'self_enc'), self._mlp(options, 'opt_enc')])
        keys_ok = np.concatenate([[True], mask])
        n, dh = len(x), D_MODEL // HEADS

        h = self._layer_norm(x, 'ln1')
        q, k, v = (self._linear(h, name).reshape(n, HEADS, dh).transpose(1, 0, 2)
                   for name in ('q', 'k', 'v'))
        scores = q @ k.transpose(0, 2, 1) / np.sqrt(dh)
        scores = np.where(keys_ok[None, None, :], scores, NEG)
        scores = np.exp(scores - scores.max(-1, keepdims=True))
        att = (scores / scores.sum(-1, keepdims=True)) @ v              # (H, T, dh)
        x = x + self._linear(att.transpose(1, 0, 2).reshape(n, D_MODEL), 'o')
        x = x + self._linear(_relu(self._linear(self._layer_norm(x, 'ln2'), 'ff1')), 'ff2')

        out = self._mlp(x[1:], 'head')[:, 0]
        return np.where(mask, out, NEG)

    def act(self, obs) -> int:
        z = self.logits(obs.options, obs.mask, obs.self_features)
        if not self.sample:
            return int(np.argmax(z))
        p = np.exp(z - z.max())
        return int(self.rng.choice(len(z), p=p / p.sum()))

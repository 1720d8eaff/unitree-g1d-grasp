"""Short-history failure/progress encoder and residual action adapter."""
import torch
from torch import nn
from torch.nn import functional as F

FAILURE_CLASSES = ("none", "visual_localization", "grasp", "execution_deviation", "target_change", "premature_termination")


class FailureAdapter(nn.Module):
    def __init__(self, state_dim: int, action_dim: int, hidden: int = 128):
        super().__init__()
        if min(state_dim, action_dim, hidden) <= 0:
            raise ValueError("dimensions must be positive")
        self.encoder = nn.GRU(state_dim, hidden, batch_first=True)
        self.failure_head = nn.Linear(hidden, len(FAILURE_CLASSES))
        self.progress_head = nn.Linear(hidden, 1)
        self.action_head = nn.Sequential(nn.Linear(hidden + action_dim, hidden), nn.ReLU(), nn.Linear(hidden, action_dim))

    def forward(self, history, base_actions):
        # history: [batch, time, state]; base_actions: [batch, horizon, action]
        if history.ndim != 3 or base_actions.ndim != 3 or history.shape[0] != base_actions.shape[0] or history.shape[1] == 0:
            raise ValueError("expected nonempty history and batched action chunks")
        _, h = self.encoder(history)
        representation = h[-1]
        context = representation.unsqueeze(1).expand(-1, base_actions.shape[1], -1)
        actions = base_actions + self.action_head(torch.cat((base_actions, context), dim=-1))
        return {"actions": actions, "failure_logits": self.failure_head(representation), "progress": torch.sigmoid(self.progress_head(representation)).squeeze(-1)}


def training_loss(output, target_actions, failure_labels, progress, *, failure_weight=1.0, progress_weight=1.0):
    """Supervised action, failure classification and progress regression loss."""
    if output["actions"].shape != target_actions.shape or progress.shape != output["progress"].shape:
        raise ValueError("target shapes do not match predictions")
    return (F.mse_loss(output["actions"], target_actions)
            + failure_weight * F.cross_entropy(output["failure_logits"], failure_labels)
            + progress_weight * F.mse_loss(output["progress"], progress))


def sampling_weights(failure_labels, hard_case_weight=3.0):
    """Return per-example weights for torch.utils.data.WeightedRandomSampler."""
    if hard_case_weight <= 0:
        raise ValueError("hard_case_weight must be positive")
    labels = torch.as_tensor(failure_labels)
    if labels.ndim != 1 or torch.any((labels < 0) | (labels >= len(FAILURE_CLASSES))):
        raise ValueError("invalid failure labels")
    return torch.where(labels == 0, 1.0, float(hard_case_weight)).double()

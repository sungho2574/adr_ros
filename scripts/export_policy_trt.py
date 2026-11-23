"""Export the mask encoder and recurrent policy step as fixed-batch ONNX models."""
import argparse
from pathlib import Path

import torch

from adr_control.policy import Policy


class Encoder(torch.nn.Module):
    def __init__(self, policy):
        super().__init__()
        self.policy = policy

    def forward(self, mask):
        return self.policy.encode_mask(mask)


class RecurrentStep(Encoder):
    def forward(self, image, vector, deter, stoch, previous, noise):
        return self.policy.forward_state(image, vector, deter, stoch, previous, noise)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bundle", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    torch.set_num_threads(1)
    policy = Policy(args.bundle, device="cpu")
    mask = torch.zeros((1, 64, 64), dtype=torch.float32)
    torch.onnx.export(
        Encoder(policy).eval(), (mask,), args.output / "encoder.onnx",
        input_names=["mask"], output_names=["image"], opset_version=17,
    )
    torch.onnx.export(
        RecurrentStep(policy).eval(),
        (policy.encode_mask(mask), torch.zeros(1, sum(policy.vec_dims)),
         policy.deter, policy.stoch, policy.prev, torch.zeros(1, policy.S, policy.C)),
        args.output / "step.onnx",
        input_names=["image", "vector", "deter", "stoch", "previous", "noise"],
        output_names=["action", "gate", "next_deter", "next_stoch"], opset_version=17,
    )


if __name__ == "__main__":
    main()

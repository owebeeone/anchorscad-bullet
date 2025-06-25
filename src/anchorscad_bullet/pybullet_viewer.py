import importlib
import inspect
import sys
import time
from typing import Any, get_args, get_origin
import pythonopenscad as posc
from anchorscad import find_all_shape_classes

import argparse
import pybullet as p
import pybullet_data
from render_bullet import APModel
import numpy as np


def _parse_value(value_str: str, target_type: Any) -> Any:
    """
    Convert a string value to a given target type.
    Supports int, float, str, bool, and lists of those types.
    """
    if target_type == inspect.Parameter.empty:
        return value_str  # No type hint, return as string

    if target_type is bool:
        return value_str.lower() in ['true', '1', 't', 'y', 'yes']

    origin = get_origin(target_type)
    if origin in [list, tuple, set, np.ndarray]:
        args = get_args(target_type)
        item_type = args[0] if args else str
        items = value_str.split(',')
        if item_type is bool:
            return [_parse_value(item, bool) for item in items]
        else:
            return [item_type(item) for item in items]
    
    try:
        return target_type(value_str)
    except (ValueError, TypeError):
        raise TypeError(f"Cannot convert '{value_str}' to {target_type}")


def initialize_pybullet() -> int:
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(
        pybullet_data.getDataPath(), 
        physicsClientId=physicsClient)
    p.setGravity(0, 0, -9.8, physicsClientId=physicsClient)
    p.loadURDF("plane.urdf", physicsClientId=physicsClient)
    return physicsClient


def visualize(physicsClient: int):

    # --- 4. Run the Simulation Loop ---
    # Set the camera to a good viewing angle
    p.resetDebugVisualizerCamera(
        cameraDistance=10, 
        cameraYaw=30, 
        cameraPitch=-25, 
        cameraTargetPosition=[0,0,0],
        physicsClientId=physicsClient)

    try:
        # Run the simulation for a fixed number of steps
        for i in range(4000):
            p.stepSimulation(physicsClientId=physicsClient)
            time.sleep(1. / 25.)
    except p.error as e:
        print(f"Error in simulation: {e}", file=sys.stderr)
        print(f"Simulation stopped.", file=sys.stderr)
        return

    p.disconnect(physicsClient)
    print("Simulation finished and disconnected.")
    

def run_simulation(
    model: APModel
) -> None:
    physicsClient = initialize_pybullet()
    
    model.to_uniform_colour_object(
        physicsClientId=physicsClient)
    
    visualize(physicsClient)

def main():
    
    parser = argparse.ArgumentParser()
    parser.add_argument("--module", type=str, default="anchorscad", required=False)
    parser.add_argument("--shape", type=str, default="Cone", required=False)
    parser.add_argument("--example", type=str, default=None, required=False,
                        help="Name of the example to render. If not provided, shape constructor arguments can be passed.")
    parser.add_argument("--part", type=str, default=None, required=False)
    parser.add_argument("--physical", type=bool, default=None, required=False)
    parser.add_argument("--material", type=str, default=None, required=False)
    
    args, unknown_args = parser.parse_known_args()

    module = importlib.import_module(args.module)
    try:
        shape_clz = getattr(module, args.shape)
    except AttributeError:
        print(f"Shape class {args.shape} not found in module {args.module}.", 
              file=sys.stderr)
        shape_names = [clz.__name__ for clz in find_all_shape_classes(module)]
        print(f"Available classes: {shape_names}", file=sys.stderr)
        raise ValueError(
            f"Shape class {args.shape} not found in module {args.module}. "
            f"Available classes: {shape_names}")

    if args.example:
        if unknown_args:
            print(f"Warning: Unknown arguments ignored when --example is used: {unknown_args}", file=sys.stderr)
        model = APModel.from_anchorscad_shape_class(
            shape_type=shape_clz,
            example_name=args.example,
            part_name=args.part,
            material_name=args.material,
            physical=args.physical)
    else:
        # Parse kwargs for shape constructor
        shape_kwargs = {}
        i = 0
        while i < len(unknown_args):
            arg = unknown_args[i]
            if arg.startswith('--'):
                key = arg[2:]
                if i + 1 < len(unknown_args) and not unknown_args[i+1].startswith('--'):
                    shape_kwargs[key] = unknown_args[i+1]
                    i += 2
                else: # Boolean flag
                    shape_kwargs[key] = 'true'
                    i += 1
            else:
                raise ValueError(f"Malformed arguments for shape constructor: {unknown_args[i:]}")

        # Inspect constructor and convert types
        try:
            sig = inspect.signature(shape_clz.__init__)
            typed_kwargs = {}
            for key, value_str in shape_kwargs.items():
                if key in sig.parameters:
                    param = sig.parameters[key]
                    typed_kwargs[key] = _parse_value(value_str, param.annotation)
                else:
                    print(f"Warning: Argument '{key}' not found in {args.shape} constructor, ignoring.", file=sys.stderr)
            
            shape_instance = shape_clz(**typed_kwargs)
            model = APModel.from_anchorscad_shape(
                shape_instance,
                part_name=args.part,
                material_name=args.material,
                physical=args.physical)
        except Exception as e:
            print(f"Error instantiating shape '{args.shape}': {e}", file=sys.stderr)
            print(f"Available constructor arguments: {list(inspect.signature(shape_clz.__init__).parameters.keys())[1:]}", file=sys.stderr)
            raise

    run_simulation(model)
    
def test_posc_main():
    # Test with two different colored shapes
    red_cube = posc.Color('red')(posc.Cube([1, 1, 1.5]))
    green_cube = posc.Color('green')(posc.Translate([0.95, 0.95, -0.15])(posc.Cube([1, 1, 1])))
    
    model = red_cube + green_cube
    render_context: posc.RenderContext = model.renderObj(posc.M3dRenderer())
    manifold = render_context.get_solid_manifold()
    
    model = APModel.from_manifold(manifold)
    
    run_simulation(model)


if __name__ == "__main__":
    import sys

    if len(sys.argv) == 1:
        pass
        sys.argv = [
            sys.argv[0],
            "--module",
            "anchorscad",
            "--shape",
            "Cone",
            "--part",
            "default",
            "--material",
            "default",
            "--",
            "--h", "3",
            "--r_top", "1",
            "--r_base", "10",
            "--fn", "64",
        ]
    main()
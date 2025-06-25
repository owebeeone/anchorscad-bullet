import importlib
import time
from typing import Callable
from anchorscad.renderer import PartMaterial
import pythonopenscad as posc
from anchorscad import Shape, find_all_shape_classes

import argparse
import pybullet as p
import pybullet_data
from render_bullet import APModel


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

    # Run the simulation for a fixed number of steps
    for i in range(4000):
        p.stepSimulation(physicsClientId=physicsClient)
        time.sleep(1. / 25.)

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
    parser.add_argument("--example", type=str, default="default", required=False)
    parser.add_argument("--part", type=str, default=None, required=False)
    parser.add_argument("--physical", type=bool, default=None, required=False)
    parser.add_argument("--material", type=str, default=None, required=False)
    args = parser.parse_args()

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

    model = APModel.from_anchorscad_shape_class(
        shape_clz,
        args.example,
        args.part,
        args.material,
        args.physical)
    
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
            "--example",
            "default",
            "--part",
            "default",
            "--material",
            "default",
        ]
    main()
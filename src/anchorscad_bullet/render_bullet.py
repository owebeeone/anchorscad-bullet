from typing import Any
from anchorscad.renderer import PartMaterial, RenderResult
import pythonopenscad as posc
from anchorscad import Shape, Maker, render, Cone
from pythonopenscad.m3dapi import M3dRenderer, RenderContextManifold
from pythonopenscad.posc_main import posc_main
import manifold3d as m3d
import numpy as np
from dataclasses import dataclass
import pybullet as p
import trimesh as tm

@dataclass
class _Part:
    vis: int
    coll: int
    mass: float
    pos: np.ndarray[tuple[int, int], np.dtype[np.float32]]
    
    
@dataclass
class APModel:
    vertices: np.ndarray[tuple[int, int], np.dtype[np.float32]]
    colours: np.ndarray[tuple[int, int], np.dtype[np.float32]]
    normals: np.ndarray[tuple[int, int], np.dtype[np.float32]]
    triangles: np.ndarray[tuple[int, int], np.dtype[np.int32]]
    centroid: np.ndarray[tuple[int], np.dtype[np.float32]]
    
    @staticmethod
    def from_manifold(
        manifold: m3d.Manifold, 
        pos_offset: int = 0, 
        color_offset: int = 3, 
        normal_offset: int = 7) -> "APModel":
        
                # Get the mesh from the manifold
        mesh: m3d.Mesh = manifold.to_mesh()

        # Extract vertex positions and triangle indices
        positions = mesh.vert_properties
        triangles = mesh.tri_verts # type: ignore
        
        model_data: np.ndarray[tuple[int, int], np.dtype[np.float32]] = \
            np.asarray(positions) # type: ignore
        triangles: np.ndarray[tuple[int, int], np.dtype[np.int32]] = \
            np.asarray(triangles) # type: ignore
            
        num_prop = max(pos_offset + 3, color_offset + 4, normal_offset + 3)

        if len(model_data) > 0 and len(model_data[0]) < num_prop:
            raise ValueError(
                f"Manifold/Mesh must have at least {num_prop} values in its property array: "
                f"{len(model_data[0])} values found"
            )
        
        
        vertices: np.ndarray[tuple[int, int], np.dtype[np.float32]] = \
            model_data[:, pos_offset:pos_offset + 3].astype(np.float32) # type: ignore
        colours: np.ndarray[tuple[int, int], np.dtype[np.float32]] = \
            model_data[:, color_offset:color_offset + 4].astype(np.float32) # type: ignore
        normals: np.ndarray[tuple[int, int], np.dtype[np.float32]] = \
            model_data[:, normal_offset:normal_offset + 3].astype(np.float32) # type: ignore
        
        tmesh = tm.Trimesh(
            vertices=vertices, 
            faces=triangles, 
            vertex_normals=normals,
            vertex_colors=colours)
        
        return APModel(
            vertices=vertices, 
            colours=colours, 
            normals=normals, 
            triangles=triangles, 
            centroid=tmesh.centroid.astype(np.float32))
    
    def first_vertex_colour(self) -> np.ndarray[tuple[int], np.dtype[np.float32]]:
        first_vertex_index: int = self.triangles[0][0]
        return self.colours[first_vertex_index]
    
    def to_uniform_colour_object(
        self, 
        colour: list[float] | np.ndarray[tuple[int], np.dtype[np.float32]] | None = None, 
        physicsClientId: int | None = None) -> int:

        if colour is None:
            colour = self.first_vertex_colour()
        else:
            colour = np.asarray(colour) # type: ignore
        
        vis_shape_id: int = p.createVisualShape(
            shapeType=p.GEOM_MESH,
            vertices=self.vertices,
            normals=self.normals,
            indices=self.triangles.flatten(),
            rgbaColor=colour,
            physicsClientId=physicsClientId
        )
        
        coll_shape = p.createCollisionShape(
            shapeType=p.GEOM_MESH, 
            vertices=self.vertices - self.centroid,
            indices=self.triangles.flatten())
        
        body_id = p.createMultiBody(
            baseMass=1,
            baseCollisionShapeIndex=coll_shape,
            baseVisualShapeIndex=vis_shape_id,
            basePosition=np.array([0, 0, 3], dtype=np.float32)
        )
        
        return body_id
    
    @staticmethod
    def from_posc_model(
        pythonscad_obj: posc.PoscBase,
        renderer: posc.M3dRenderer | None = None
        ) -> "APModel":
        
        if renderer is None:
            renderer = posc.M3dRenderer()
            
        render_context = pythonscad_obj.renderObj(renderer) # type: ignore
        
        if not isinstance(render_context, posc.RenderContextManifold):
            raise ValueError(
                f"Render context is not a manifold: {type(render_context)}"
            )
        render_context: posc.RenderContextManifold = render_context
        
        manifold: m3d.Manifold = render_context.get_solid_manifold()
        
        return APModel.from_manifold(manifold)
        
    @staticmethod
    def from_anchorscad_shape(
        shape: Shape,
        part_name: str | None = None,
        material_name: str | None = None,
        physical: bool | None = None,
        renderer: posc.M3dRenderer | None = None,
        ) -> "APModel":
        
        # Run the anchorscad renderer, this creates a number of posc shapes.
        result: RenderResult = render(shape) # type: ignore
        # Collect the posc shapes that are part of the requested part.
        posc_shapes: list[posc.PoscBase] = []
        if part_name is None:
            posc_shapes.append(result.rendered_shape)
        else:
            # Find the requested part in the rendered parts.
            if physical is None:
                keys = ((part_name, 'physical'), (part_name, 'non_physical'))
            elif physical:
                keys = ((part_name, 'physical'),)
            else:
                keys = ((part_name, 'non_physical'),)
                
            for key in keys:
                if key in result.parts:
                    posc_shapes.append(result.parts[key])
                    break
                
            if len(posc_shapes) == 0:
                if physical is None:
                    raise ValueError(
                        f"Part {part_name} not found in rendered parts. "
                        f"Available parts: {[name for name, _ in result.parts.keys()]}, "
                        f"(Keys: {list(result.parts.keys())})")
                else:
                    raise ValueError(
                        f"Part {part_name} as {keys[0][1]} not found in rendered parts. "
                        f"Available (part, physical) shapes: "
                        f"{[key for key in result.parts.keys()]}")


        # Collect the materials found in the shape to be used for
        # error messages if the requested material is not found.
        materials_found: set[str] = set()
        result_shapes: list[posc.PoscBase] = []
        if material_name is not None:
            for posc_shape in posc_shapes:
                if isinstance(posc_shape, posc.LazyUnion):
                    for child in posc_shape.children():
                        part_material = child.getDescriptor()
                        if isinstance(part_material, PartMaterial):
                            name = part_material.get_material().name
                            materials_found.add(name)
                            if name == material_name:
                                result_shapes.append(child)
            
            if len(result_shapes) == 0:
                if len(materials_found) == 0:
                    raise ValueError(
                        f"Material '{material_name}' requested but no materials found.")
                else:
                    raise ValueError(
                        f"Material {material_name} not found in shape. "
                        f"Available materials: {materials_found}")
        else:
            result_shapes: list[posc.PoscBase] = posc_shapes
    
        result_shape = result_shapes[0] \
            if len(result_shapes) == 1 \
            else posc.LazyUnion()(*result_shapes)
            
        return APModel.from_posc_model(result_shape, renderer)
    
    @staticmethod
    def from_anchorscad_shape_class(
        shape_type: type[Shape],
        example_name: str = "default",
        with_anchors: bool = False,
        part_name: str | None = None,
        material_name: str | None = None,
        physical: bool | None = None,
        renderer: posc.M3dRenderer | None = None
        ) -> "APModel":
        
        try:
            result: tuple[Maker, Shape] = shape_type.example(example_name)
        except KeyError:
            if hasattr(shape_type, 'EXAMPLES_EXTENDED'):
                extended_examples: dict[str, Any] = shape_type.EXAMPLES_EXTENDED # type: ignore
                available_examples = ['default'] + list(extended_examples.keys())
            else:
                available_examples = ['default']
            raise ValueError(f"Example {example_name} not found in shape {shape_type.__name__}. "
                             f"Available examples: {available_examples}")

        index = 0 if with_anchors else 1
        return APModel.from_anchorscad_shape(
            result[index], 
            part_name, 
            material_name, 
            physical,
            renderer)
    
    

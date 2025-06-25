# anchorscad-bullet

This library provides tools to convert `anchorscad` and `pythonopenscad` models into `pybullet` objects for physics simulation.

## Modules

### `render_bullet.py`

This module is the core converter. It takes models from `anchorscad` or `pythonopenscad` and converts them into a format that can be used with `pybullet`.

Key conversion functions in the `APModel` class:
- `from_manifold`: Converts a `manifold3d.Manifold` object.
- `from_posc_model`: Converts a `pythonopenscad.PoscBase` object.
- `from_anchorscad_shape`: Converts an `anchorscad.Shape` instance.
- `from_anchorscad_shape_class`: Converts an `anchorscad.Shape` class by using one of its examples.

### `pybullet_viewer.py`

This module provides a simple viewer to display and simulate the converted models using `pybullet`. It can be run as a script to view specific models.

#### Usage

You can run the viewer from the command line to display a model from a specific module.

```bash
python -m anchorscad_bullet.pybullet_viewer --module <module_name> --shape <shape_class_name> [options]
```

**Arguments:**
- `--module`: The Python module containing the `anchorscad` shape (e.g., `anchorscad.examples.basic_models`).
- `--shape`: The name of the `Shape` class to render.
- `--example`: The name of the example to use for the shape class (default: `default`).
- `--part`: The name of the part to render.
- `--material`: The name of the material to render.
- `--physical`: Whether to render the physical or non-physical version of the part.

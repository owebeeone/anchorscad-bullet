import unittest
from anchorscad import Sphere
from anchorscad_bullet.render_bullet import APModel
import pybullet as p

class TestBulletConverter(unittest.TestCase):

    def setUp(self):
        """
        Set up a direct-mode pybullet server before each test.
        """
        self.physics_client = p.connect(p.DIRECT)

    def tearDown(self):
        """
        Disconnect from the pybullet server after each test.
        """
        p.disconnect(self.physics_client)

    def test_sphere_conversion(self):
        """
        Tests the conversion of a default physical anchorscad Sphere to an APModel.
        """
        model = APModel.from_anchorscad_shape_class(
            shape_type=Sphere,
            example_name="default",
            physical=True
        )

        self.assertIsNotNone(model)
        self.assertTrue(len(model.vertices) > 0)
        self.assertTrue(len(model.triangles) > 0)
        self.assertIsNotNone(model.centroid)
        
    def test_to_uniform_colour_object(self):
        """
        Tests creating a pybullet object from an APModel.
        """
        model = APModel.from_anchorscad_shape_class(
            shape_type=Sphere,
            example_name="default",
            physical=True
        )
        
        body_id = model.to_uniform_colour_object(physicsClientId=self.physics_client)
        
        self.assertIsInstance(body_id, int)
        self.assertGreaterEqual(body_id, 0)
        # Verify that one body has been added to the simulation
        self.assertEqual(p.getNumBodies(self.physics_client), 1)

if __name__ == '__main__':
    unittest.main()

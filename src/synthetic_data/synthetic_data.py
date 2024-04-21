import numpy as np
from PIL import Image, ImageDraw
from src.fetch_nav_init.fetch_nav_init import FetchNavInit

class SyntheticData:
    def __init__(self, fetch_nav_init: FetchNavInit):
        import omni.syntheticdata as sdh
        import omni.replicator.core as rep

        self._sdh = sdh
        self._rp = rep.create.render_product(fetch_nav_init.camera_prim_path, (1280, 720))
        self._bbox_annot = rep.AnnotatorRegistry.get_annotator("bounding_box_2d_tight", init_params={"semanticTypes": ["class"]})
        self._bbox_annot.attach(self._rp)
        self._camera_params = rep.annotators.get("CameraParams").attach(self._rp).get_data()

    def bbox_2d_data(self, image):
        data = self._bbox_annot.get_data()
        print(data)
        draw = ImageDraw.Draw(image)

        for box in data['data']:
            class_id, x_min, y_min, x_max, y_max, _ = box
            draw.rectangle([x_min, y_min, x_max, y_max], outline=(255, 0, 0), width=2)

        del draw  # Release the drawing context

        # Save the image using Pillow
        return image

    def bbox_3d_data(self, img):  # assumes a single viewport attached
        data = self._bbox_annot.get_data()
        width, height = 512, 512
        bbox3ds = data["data"]

        # Get 3D BBOX corners
        corners_3d = self._sdh.helpers.get_bbox_3d_corners(bbox3ds)
        corners_3d = corners_3d.reshape(-1, 3)

        # Project to image space
        corners_2d = self.world_to_image_pinhole(corners_3d, self._camera_params)
        corners_2d *= np.array([[width, height]])

        # Draw corners on image
        self.draw_points(img, corners_2d)

        return img

    def world_to_image_pinhole(self, world_points, camera_params):
        # Project corners to image space (assumes pinhole camera model)
        # print(camera_params)
        proj_mat = camera_params["cameraProjection"].reshape(4, 4)
        view_mat = camera_params["cameraViewTransform"].reshape(4, 4)
        view_proj_mat = np.dot(view_mat, proj_mat)
        world_points_homo = np.pad(world_points, ((0, 0), (0, 1)), constant_values=1.0)
        tf_points = np.dot(world_points_homo, view_proj_mat)
        tf_points = tf_points / (tf_points[..., -1:])
        return 0.5 * (tf_points[..., :2] + 1)

    def draw_points(self, img, image_points, radius=5):
        width, height = img.size
        draw = ImageDraw.Draw(img)
        image_points[:, 1] = height - image_points[:, 1]
        for point in image_points:
            xy = (point[0] - radius, point[1] - radius, point[0] + radius, point[1] + radius)
            draw.ellipse(xy, fill=(255, 0, 0), outline=(0, 0, 0))
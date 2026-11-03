"""Render only the wheel meshes exported by wheel_test.py."""
from pathlib import Path
import mujoco
from PIL import Image
from render import build_xml


def main():
    out = Path(__file__).with_name('out')
    model = mujoco.MjModel.from_xml_string(build_xml(out / 'wheel_render_stl'))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    renderer = mujoco.Renderer(model, 1080, 1440)
    views = [('wheel_6804', (0, 0, .07), .36, 155, -22),
             ('wheel_6804_inside', (0, 0, .07), .36, 25, -22),
             ('wheel_6804_clearance', (-.018, .052, .0913), .14, 155, -25),
             ('wheel_6804_web_fillet', (-.018, .052, .0913), .14, 155, -25)]
    for name, lookat, distance, az, el in views:
        cam = mujoco.MjvCamera()
        cam.lookat[:] = lookat
        cam.distance, cam.azimuth, cam.elevation = distance, az, el
        renderer.update_scene(data, camera=cam)
        path = out / 'renders' / f'{name}.png'
        path.parent.mkdir(exist_ok=True)
        Image.fromarray(renderer.render()).save(path)
        print(path)
    renderer.close()
    web_meshes = out / 'wheel_web_stl'
    if web_meshes.exists():
        model = mujoco.MjModel.from_xml_string(build_xml(web_meshes))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        renderer = mujoco.Renderer(model, 1080, 1440)
        for name, az, el in [('wheel_web_inside', 25, -20), ('wheel_web_underside', 25, 20)]:
            cam = mujoco.MjvCamera()
            cam.lookat[:] = (-.018, .039, .092)
            cam.distance, cam.azimuth, cam.elevation = .14, az, el
            renderer.update_scene(data, camera=cam)
            path = out / 'renders' / f'{name}.png'
            Image.fromarray(renderer.render()).save(path)
            print(path)
        # Hide rollers in this review view to expose the filled web underside.
        meshes = sorted(web_meshes.glob('*.stl'))
        for geom in range(model.ngeom):
            mesh = model.geom_dataid[geom]
            if mesh >= 0 and not meshes[mesh].name.startswith(('petg_orange', 'fastener')):
                model.geom_rgba[geom, 3] = 0
        cam = mujoco.MjvCamera()
        cam.lookat[:] = (-.024, .038, .092)
        cam.distance, cam.azimuth, cam.elevation = .095, 85, -20
        renderer.update_scene(data, camera=cam)
        path = out / 'renders' / 'wheel_web_join.png'
        Image.fromarray(renderer.render()).save(path)
        print(path)
        renderer.close()


if __name__ == '__main__':
    main()

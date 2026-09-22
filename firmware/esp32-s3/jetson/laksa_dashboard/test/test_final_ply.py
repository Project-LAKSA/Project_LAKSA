import struct

import numpy as np

from laksa_dashboard.cockpit_server import _read_official_ply_preview, _transform_xyz


def test_official_binary_ply_is_bounded_and_colored(tmp_path):
    path = tmp_path / "map_cloud_cloud.ply"
    header = """ply
format binary_little_endian 1.0
element vertex 20
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
property float nx
property float ny
property float nz
property float curvature
element face 0
end_header
""".encode("ascii")
    vertices = b"".join(
        struct.pack("<fffBBBffff", float(i), i / 2.0, 1.0, i, 20, 30, 0.0, 0.0, 1.0, 0.0)
        for i in range(20)
    )
    path.write_bytes(header + vertices)

    payload = _read_official_ply_preview(path, 5)

    assert payload["source"] == "final_optimized_rgbd_map"
    assert payload["source_points"] == 20
    assert payload["sent_points"] == 5
    assert payload["downsampled"] is True
    assert len(payload["points"]) == 15
    assert len(payload["colors"]) == 15


def test_live_cloud_rigid_transform_outputs_map_coordinates():
    points = np.asarray(((1.0, 0.0, 0.0), (0.0, 1.0, 2.0)))
    # +90 degrees about Z followed by a map-frame translation.
    half = np.sqrt(0.5)
    transformed = _transform_xyz(points, (2.0, 3.0, 4.0), (0.0, 0.0, half, half))
    np.testing.assert_allclose(
        transformed,
        np.asarray(((2.0, 4.0, 4.0), (1.0, 3.0, 6.0))),
        atol=1.0e-9,
    )

import math

from Lcode.inventory_planner import InventoryPlanner, WaypointKind
from Lcode.warehouse_model import FaceId, FlightPoint, WarehouseConfig, WarehouseModel


def test_coordinate_axes_and_confirmed_lower_bypass():
    config = WarehouseConfig()
    assert config.flight_y(config.start_u_m + 1.0) == 1.0
    assert config.flight_x(config.start_v_m + 1.0) == -1.0
    assert config.lower_bypass_x_m == 0.15


def test_builds_24_unique_slots_and_expected_face_directions():
    model = WarehouseModel()
    assert len(model.slots) == 24
    assert set(model.slots) == {
        f"{face}{number}" for face in "ABCD" for number in range(1, 7)
    }
    points = {(slot.point.x, slot.point.y, slot.point.z) for slot in model.slots.values()}
    assert len(points) == 24
    assert model.faces[FaceId.A].look_y_sign == +1
    assert model.faces[FaceId.B].look_y_sign == -1
    assert model.faces[FaceId.C].look_y_sign == +1
    assert model.faces[FaceId.D].look_y_sign == -1


def test_opposite_faces_reverse_visible_column_labels():
    model = WarehouseModel()
    assert model.slots["A1"].point.x == -1.75
    assert model.slots["A3"].point.x == -0.75
    assert model.slots["B1"].point.x == -0.75
    assert model.slots["B3"].point.x == -1.75


def test_full_plan_has_all_slots_once_and_fixed_gimbal_per_face():
    planner = InventoryPlanner()
    route = planner.plan_full_inventory()
    inspections = [p for p in route if p.kind == WaypointKind.INSPECT]
    assert len(inspections) == 24
    assert len({p.slot_label for p in inspections}) == 24
    assert route[0].kind == WaypointKind.TAKEOFF
    assert route[-1].kind == WaypointKind.LAND

    set_gimbal = [p for p in route if p.kind == WaypointKind.SET_GIMBAL]
    assert [p.face for p in set_gimbal] == list(InventoryPlanner.FULL_FACE_ORDER)
    assert [p.gimbal_angle_deg for p in set_gimbal] == [0.0, 180.0, 0.0, 180.0]
    assert 0 < planner.route_length_m(route) < 22


def test_face_scan_starts_at_previous_face_height_and_bypass_keeps_it():
    planner = InventoryPlanner()
    route = planner.plan_full_inventory()
    first_inspect_z = [
        next(p.point.z for p in route if p.kind == WaypointKind.INSPECT and p.face == face)
        for face in InventoryPlanner.FULL_FACE_ORDER
    ]
    assert first_inspect_z == [1.4, 1.0, 1.4, 1.0]

    lower_bypass_z = [
        p.point.z
        for p in route
        if p.kind == WaypointKind.TRANSIT and math.isclose(p.point.x, 0.15)
    ]
    assert lower_bypass_z == [1.0, 1.0, 1.0, 1.0]


def test_full_plan_uses_nearest_bypass_for_reversed_scan_endpoints():
    planner = InventoryPlanner()
    route = planner.plan_full_inventory()
    transit_x = [p.point.x for p in route if p.kind == WaypointKind.TRANSIT]
    assert any(math.isclose(x, 0.15) for x in transit_x)
    assert not any(math.isclose(x, -2.65) for x in transit_x)


def test_target_on_far_side_uses_confirmed_lower_bypass_when_shorter():
    planner = InventoryPlanner()
    route = planner.plan_target("D3")
    transit_x = [p.point.x for p in route if p.kind == WaypointKind.TRANSIT]
    assert any(math.isclose(x, 0.15) for x in transit_x)
    assert [p.slot_label for p in route if p.kind == WaypointKind.INSPECT] == ["D3"]


def test_same_x_crossing_uses_upper_bypass_when_it_is_shorter():
    planner = InventoryPlanner()
    start_slot = planner.model.slots["A1"]
    end_slot = planner.model.slots["B3"]
    start = FlightPoint(start_slot.point.x, start_slot.point.y, planner.model.config.cruise_z_m)
    end_point = FlightPoint(end_slot.point.x, end_slot.point.y, planner.model.config.cruise_z_m)

    assert math.isclose(start.x, end_point.x)
    assert math.isclose(planner._choose_bypass_x(start, end_point), -2.65)


def test_invalid_bypass_inside_shelf_is_rejected():
    try:
        WarehouseConfig(lower_bypass_x_m=-0.50)
    except ValueError as exc:
        assert "下端通道" in str(exc)
    else:
        raise AssertionError("货架范围内的下端通道应被拒绝")

use crate::rigid_body::BodyType;

use super::CcdIslandLayout;

#[test]
fn independent_dynamics_against_one_static_body_form_separate_islands() {
    let mut body_types = vec![BodyType::Dynamic; 64];
    body_types.push(BodyType::Static);
    let mut continuous = vec![true; 64];
    continuous.push(false);
    let candidates = (0..64).map(|body| (body, 64)).collect::<Vec<_>>();
    let mut layout = CcdIslandLayout::new();

    layout
        .build(&body_types, &continuous, &candidates, false)
        .expect("build CCD islands");

    assert_eq!(layout.island_count(), 64);
    for candidate in 0..64 {
        assert_eq!(layout.candidate_island(candidate), Some(candidate));
        assert_eq!(layout.candidates(candidate), Some(&[candidate][..]));
    }
}

#[test]
fn connected_dynamic_chain_forms_one_island() {
    let body_types = [BodyType::Dynamic; 4];
    let continuous = [true; 4];
    let candidates = [(0, 1), (1, 2), (2, 3)];
    let mut layout = CcdIslandLayout::new();

    layout
        .build(&body_types, &continuous, &candidates, false)
        .expect("build CCD islands");

    assert_eq!(layout.island_count(), 1);
    assert_eq!(layout.candidates(0), Some(&[0, 1, 2][..]));
}

#[test]
fn multiple_colliders_on_one_body_share_its_island() {
    let body_types = [BodyType::Dynamic, BodyType::Static];
    let continuous = [true, false];
    let candidates = [(0, 1), (0, 1)];
    let mut layout = CcdIslandLayout::new();

    layout
        .build(&body_types, &continuous, &candidates, false)
        .expect("build CCD islands");

    assert_eq!(layout.island_count(), 1);
    assert_eq!(layout.candidates(0), Some(&[0, 1][..]));
}

#[test]
fn shared_kinematic_surface_does_not_join_dynamic_islands() {
    let body_types = [BodyType::Dynamic, BodyType::Dynamic, BodyType::Kinematic];
    let continuous = [true, true, false];
    let candidates = [(0, 2), (1, 2)];
    let mut layout = CcdIslandLayout::new();

    layout
        .build(&body_types, &continuous, &candidates, false)
        .expect("build CCD islands");

    assert_eq!(layout.island_count(), 2);
    assert_eq!(layout.candidate_island(0), Some(0));
    assert_eq!(layout.candidate_island(1), Some(1));
}

#[test]
fn no_ccd_candidates_produce_no_islands() {
    let body_types = [BodyType::Dynamic, BodyType::Static];
    let continuous = [true, false];
    let mut layout = CcdIslandLayout::new();

    layout
        .build(&body_types, &continuous, &[], false)
        .expect("build empty CCD islands");

    assert_eq!(layout.island_count(), 0);
    assert_eq!(layout.candidates(0), None);
}

#[test]
fn island_and_candidate_order_is_stable_across_builds() {
    let body_types = [
        BodyType::Dynamic,
        BodyType::Dynamic,
        BodyType::Dynamic,
        BodyType::Static,
    ];
    let continuous = [true, true, true, false];
    let candidates = [(2, 3), (0, 3), (0, 1)];
    let mut layout = CcdIslandLayout::new();

    layout
        .build(&body_types, &continuous, &candidates, false)
        .expect("first build");
    let first = (0..layout.island_count())
        .map(|island| layout.candidates(island).map(<[usize]>::to_vec))
        .collect::<Vec<_>>();

    layout
        .build(&body_types, &continuous, &candidates, false)
        .expect("second build");
    let second = (0..layout.island_count())
        .map(|island| layout.candidates(island).map(<[usize]>::to_vec))
        .collect::<Vec<_>>();

    assert_eq!(first, second);
    assert_eq!(layout.candidate_island(0), Some(1));
    assert_eq!(layout.candidate_island(1), Some(0));
    assert_eq!(layout.candidate_island(2), Some(0));
}

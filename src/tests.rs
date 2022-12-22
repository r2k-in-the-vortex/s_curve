mod tests {
    use crate::{
        s_curve_generator, Derivative, SCurveConstraints, SCurveInput, SCurveParameters,
        SCurveStartConditions,
    };

    fn test_continuous(input: &SCurveInput, d: Derivative, constraining_param: f64) -> bool {
        let near_equal = |a: f64, b: f64, epsilon: f64| f64::abs(a - b) < epsilon;
        let (params, s_curve) = s_curve_generator(input, d);
        let mut p0 = s_curve(0.0);
        let datapoints = 1000;
        let margin = 100;
        let divisor = datapoints as f64;
        let e = constraining_param * 1.1 * params.time_intervals.total_duration() / divisor;
        for i in 0 - margin..(datapoints + 1) + margin {
            let p1 = s_curve(i as f64 * params.time_intervals.total_duration() / divisor);
            if near_equal(p0, p1, e) {
                p0 = p1;
            } else {
                println!("Out of limits, {p0}, {p1}, difference lim {e}");
                return false;
            }
        }
        return true;
    }

    fn test_continuous_pva(input: &SCurveInput) -> bool {
        if !test_continuous(input, Derivative::Position, input.constraints.max_velocity) {
            println!("Position not continuous");
            return false;
        }
        if !test_continuous(
            input,
            Derivative::Velocity,
            input.constraints.max_acceleration,
        ) {
            println!("Velocity not continuous");
            return false;
        }
        if !test_continuous(input, Derivative::Acceleration, input.constraints.max_jerk) {
            println!("Acceleration not continuous");
            return false;
        }
        return true;
    }

    #[test]
    fn timings_3_9() {
        let constraints = SCurveConstraints {
            max_jerk: 30.,
            max_acceleration: 10.0,
            max_velocity: 5.,
        };
        let start_conditions = SCurveStartConditions {
            q0: 0.,
            q1: 10.,
            v0: 1.,
            v1: 0.,
        };
        let input = SCurveInput {
            constraints,
            start_conditions,
        };
        let times = input.calc_intervals();
        let near_equal = |a: f64, b: f64, epsilon: f64| f64::abs(a - b) < epsilon;
        assert!(near_equal(times.t_a, 0.7333, 0.001));
        assert!(near_equal(times.t_v, 1.1433, 0.001));
        assert!(near_equal(times.t_d, 0.8333, 0.001));
        assert!(near_equal(times.t_j1, 0.333, 0.001));
        assert!(near_equal(times.t_j2, 0.333, 0.001));
    }

    #[test]
    fn timings_3_10() {
        let constraints = SCurveConstraints {
            max_jerk: 30.,
            max_acceleration: 10.0,
            max_velocity: 10.,
        };
        let start_conditions = SCurveStartConditions {
            q0: 0.,
            q1: 10.,
            v0: 1.,
            v1: 0.,
        };
        let input = SCurveInput {
            constraints,
            start_conditions,
        };
        let times = input.calc_intervals();
        let near_equal = |a: f64, b: f64, epsilon: f64| f64::abs(a - b) < epsilon;
        assert!(near_equal(times.t_a, 1.0747, 0.001));
        assert!(near_equal(times.t_v, 0., 0.001));
        assert!(near_equal(times.t_d, 1.1747, 0.001));
        assert!(near_equal(times.t_j1, 0.333, 0.001));
        assert!(near_equal(times.t_j2, 0.333, 0.001));
    }

    #[test]
    fn timings_3_11() {
        let constraints = SCurveConstraints {
            max_jerk: 30.,
            max_acceleration: 10.0,
            max_velocity: 10.,
        };
        let start_conditions = SCurveStartConditions {
            q0: 0.,
            q1: 10.,
            v0: 7.,
            v1: 0.,
        };
        let input = SCurveInput {
            constraints,
            start_conditions,
        };
        let times = input.calc_intervals();
        let near_equal = |a: f64, b: f64, epsilon: f64| f64::abs(a - b) < epsilon;
        assert!(near_equal(times.t_a, 0.4666, 0.001));
        assert!(near_equal(times.t_v, 0., 0.001));
        assert!(near_equal(times.t_d, 1.4718, 0.001));
        assert!(near_equal(times.t_j1, 0.2312, 0.001));
        assert!(near_equal(times.t_j2, 0.2321, 0.001));
    }

    #[test]
    fn timings_3_12() {
        let constraints = SCurveConstraints {
            max_jerk: 30.,
            max_acceleration: 10.0,
            max_velocity: 10.,
        };
        let start_conditions = SCurveStartConditions {
            q0: 0.,
            q1: 10.,
            v0: 7.5,
            v1: 0.,
        };
        let input = SCurveInput {
            constraints,
            start_conditions,
        };
        let times = input.calc_intervals();
        let near_equal = |a: f64, b: f64, epsilon: f64| f64::abs(a - b) < epsilon;
        assert!(near_equal(times.t_a, 0., 0.001));
        assert!(near_equal(times.t_v, 0., 0.001));
        assert!(near_equal(times.t_d, 2.6667, 0.001));
        assert!(near_equal(times.t_j1, 0., 0.001));
        assert!(near_equal(times.t_j2, 0.0973, 0.001));
    }

    #[test]
    fn simple_curve() {
        let constraints = SCurveConstraints {
            max_jerk: 30.,
            max_acceleration: 10.0,
            max_velocity: 5.,
        };
        let start_conditions = SCurveStartConditions {
            q0: 0.,
            q1: 10.,
            v0: 1.,
            v1: 0.,
        };
        let input = SCurveInput {
            constraints,
            start_conditions,
        };
        let s_curve_tmp = s_curve_generator(&input, Derivative::Position);
        let s_curve = s_curve_tmp.1;
        let params = s_curve_tmp.0;
        for i in 0..101 {
            println!(
                "{}",
                s_curve(i as f64 * params.time_intervals.total_duration() / 100.)
            );
        }
    }

    #[test]
    fn simple_curve_reverse() {
        let constraints = SCurveConstraints {
            max_jerk: 3.,
            max_acceleration: 2.0,
            max_velocity: 3.,
        };
        let start_conditions = SCurveStartConditions {
            q0: 10.,
            q1: 0.,
            v0: 0.,
            v1: 0.,
        };
        let input = SCurveInput {
            constraints,
            start_conditions,
        };
        let times = input.calc_intervals();

        assert_eq!(times.total_duration(), 5.5);
    }

    #[test]
    fn simple_curve_reverse_no_phase_3() {
        let constraints = SCurveConstraints {
            max_jerk: 3.,
            max_acceleration: 2.0,
            max_velocity: 3.,
        };
        let start_conditions = SCurveStartConditions {
            q0: 5.,
            q1: 0.,
            v0: 0.,
            v1: 0.,
        };
        let input = SCurveInput {
            constraints,
            start_conditions,
        };
        let times = input.calc_intervals();

        assert_eq!(times.total_duration(), 3.8984532382775527);
    }

    #[test]
    fn test_is_a_min_not_reached() {
        let constraints = SCurveConstraints {
            max_jerk: 0.03,
            max_acceleration: 2.0,
            max_velocity: 3.,
        };
        let start_conditions = SCurveStartConditions {
            q0: 10.,
            q1: 0.,
            v0: 0.,
            v1: 0.,
        };
        let input = SCurveInput {
            constraints,
            start_conditions,
        };

        assert!(test_continuous_pva(&input));
    }

    #[test]
    fn test_is_max_acceleration_not_reached() {
        let constraints = SCurveConstraints {
            max_jerk: 3.,
            max_acceleration: 2.0,
            max_velocity: 3.,
        };
        let start_conditions = SCurveStartConditions {
            q0: 1.,
            q1: 0.,
            v0: 0.,
            v1: 0.,
        };
        let input = SCurveInput {
            constraints,
            start_conditions,
        };

        assert!(test_continuous_pva(&input));
    }

    #[test]
    fn vlim_move_pos_v0_right_direction() {
        let constraints = SCurveConstraints {
            max_jerk: 3.,
            max_acceleration: 2.0,
            max_velocity: 3.,
        };
        let start_conditions = SCurveStartConditions {
            q0: 0.,
            q1: 10.,
            v0: 1.,
            v1: 0.,
        };
        let input = SCurveInput {
            constraints,
            start_conditions,
        };
        let times = input.calc_intervals();
        let params = SCurveParameters::new(&times, &input);

        assert_eq!(params.v_lim, 3.0);
    }

    #[test]
    fn vlim_move_pos_v0_wrong_direction() {
        let constraints = SCurveConstraints {
            max_jerk: 3.,
            max_acceleration: 2.0,
            max_velocity: 3.,
        };
        let start_conditions = SCurveStartConditions {
            q0: 0.,
            q1: 10.,
            v0: -1.,
            v1: 0.,
        };
        let input = SCurveInput {
            constraints,
            start_conditions,
        };
        let times = input.calc_intervals();
        let params = SCurveParameters::new(&times, &input);

        assert_eq!(params.v_lim, 3.0);
    }

    #[test]
    fn vlim_move_neg_v0_right_direction() {
        let constraints = SCurveConstraints {
            max_jerk: 3.,
            max_acceleration: 2.0,
            max_velocity: 3.,
        };
        let start_conditions = SCurveStartConditions {
            q0: 10.,
            q1: 0.,
            v0: -1.,
            v1: 0.,
        };
        let input = SCurveInput {
            constraints,
            start_conditions,
        };
        let times = input.calc_intervals();
        let params = SCurveParameters::new(&times, &input);

        assert_eq!(params.v_lim, 3.0);
    }

    #[test]
    fn vlim_move_neg_v0_wrong_direction() {
        let constraints = SCurveConstraints {
            max_jerk: 3.,
            max_acceleration: 2.0,
            max_velocity: 3.,
        };
        let start_conditions = SCurveStartConditions {
            q0: 10.,
            q1: 0.,
            v0: 1.,
            v1: 0.,
        };
        let input = SCurveInput {
            constraints,
            start_conditions,
        };
        let times = input.calc_intervals();
        let params = SCurveParameters::new(&times, &input);

        assert_eq!(params.v_lim, 3.0);
    }

    #[test]
    fn t_v_pos_move_v0_wrong_direction() {
        let constraints = SCurveConstraints {
            max_jerk: 3.,
            max_acceleration: 2.0,
            max_velocity: 3.,
        };
        let start_conditions = SCurveStartConditions {
            q0: 0.,
            q1: 10.,
            v0: -1.,
            v1: 0.,
        };
        let input = SCurveInput {
            constraints,
            start_conditions,
        };
        let times = input.calc_intervals();

        assert_eq!(times.t_v, 1.3611111111111114);
    }

    #[test]
    fn t_v_neg_move_v0_wrong_direction() {
        let constraints = SCurveConstraints {
            max_jerk: 3.,
            max_acceleration: 2.0,
            max_velocity: 3.,
        };
        let start_conditions = SCurveStartConditions {
            q0: 10.,
            q1: 0.,
            v0: 1.,
            v1: 0.,
        };
        let input = SCurveInput {
            constraints,
            start_conditions,
        };
        let times = input.calc_intervals();

        assert_eq!(times.t_v, 1.3611111111111114);
    }

    #[test]
    fn pos_move_v0_wrong_direction() {
        let constraints = SCurveConstraints {
            max_jerk: 3.,
            max_acceleration: 2.0,
            max_velocity: 3.,
        };
        let start_conditions = SCurveStartConditions {
            q0: 0.,
            q1: 10.,
            v0: -1.,
            v1: 0.,
        };
        let input = SCurveInput {
            constraints,
            start_conditions,
        };
        let times = input.calc_intervals();

        assert_eq!(times.total_duration(), 6.194444444444445);
    }

    #[test]
    fn neg_move_v0_wrong_direction() {
        let constraints = SCurveConstraints {
            max_jerk: 3.,
            max_acceleration: 2.0,
            max_velocity: 3.,
        };
        let start_conditions = SCurveStartConditions {
            q0: 10.,
            q1: 0.,
            v0: 1.,
            v1: 0.,
        };
        let input = SCurveInput {
            constraints,
            start_conditions,
        };
        let times = input.calc_intervals();

        assert_eq!(times.total_duration(), 6.194444444444445);
    }

    #[test]
    fn continuous_simple() {
        let constraints = SCurveConstraints {
            max_jerk: 3.,
            max_acceleration: 2.0,
            max_velocity: 3.,
        };
        let start_conditions = SCurveStartConditions {
            q0: 0.,
            q1: 10.,
            v0: 0.,
            v1: 0.,
        };
        let input = SCurveInput {
            constraints,
            start_conditions,
        };

        assert!(test_continuous_pva(&input));
    }

    #[test]
    fn continuous_no_accel_pos() {
        let constraints = SCurveConstraints {
            max_jerk: 3.,
            max_acceleration: 2.0,
            max_velocity: 3.,
        };
        let start_conditions = SCurveStartConditions {
            q0: 0.,
            q1: 10.,
            v0: 3.,
            v1: 3.,
        };
        let input = SCurveInput {
            constraints,
            start_conditions,
        };

        assert!(test_continuous_pva(&input));
    }

    #[test]
    fn continuous_no_accel_neg() {
        let constraints = SCurveConstraints {
            max_jerk: 3.,
            max_acceleration: 2.0,
            max_velocity: 3.,
        };
        let start_conditions = SCurveStartConditions {
            q0: 10.,
            q1: 0.,
            v0: -3.,
            v1: -3.,
        };
        let input = SCurveInput {
            constraints,
            start_conditions,
        };

        assert!(test_continuous_pva(&input));
    }

    #[test]
    fn continuous_pos_start_at_max_vel() {
        let constraints = SCurveConstraints {
            max_jerk: 3.,
            max_acceleration: 2.0,
            max_velocity: 3.,
        };
        let start_conditions = SCurveStartConditions {
            q0: 0.,
            q1: 10.,
            v0: 3.,
            v1: 0.,
        };
        let input = SCurveInput {
            constraints,
            start_conditions,
        };

        assert!(test_continuous_pva(&input));
    }

    #[test]
    fn continuous_neg_start_at_max_vel() {
        let constraints = SCurveConstraints {
            max_jerk: 3.,
            max_acceleration: 2.0,
            max_velocity: 3.,
        };
        let start_conditions = SCurveStartConditions {
            q0: 10.,
            q1: 0.,
            v0: -3.,
            v1: 0.,
        };
        let input = SCurveInput {
            constraints,
            start_conditions,
        };

        assert!(test_continuous_pva(&input));
    }

    #[test]
    fn continuous_nonzero_v1() {
        let constraints = SCurveConstraints {
            max_jerk: 3.,
            max_acceleration: 2.0,
            max_velocity: 3.,
        };
        let start_conditions = SCurveStartConditions {
            q0: 0.,
            q1: 10.,
            v0: 0.,
            v1: 1.,
        };
        let input = SCurveInput {
            constraints,
            start_conditions,
        };

        assert!(test_continuous_pva(&input));
    }

    #[test]
    fn continuous_neg_nonzero_v1() {
        let constraints = SCurveConstraints {
            max_jerk: 3.,
            max_acceleration: 2.0,
            max_velocity: 3.,
        };
        let start_conditions = SCurveStartConditions {
            q0: 10.,
            q1: 0.,
            v0: 0.,
            v1: -1.,
        };
        let input = SCurveInput {
            constraints,
            start_conditions,
        };

        assert!(test_continuous_pva(&input));
    }

    #[test]
    fn continuous_neg_nonzero_v1_wrongdirection() {
        let constraints = SCurveConstraints {
            max_jerk: 3.,
            max_acceleration: 2.0,
            max_velocity: 3.,
        };
        let start_conditions = SCurveStartConditions {
            q0: 10.,
            q1: 0.,
            v0: 0.,
            v1: 1.,
        };
        let input = SCurveInput {
            constraints,
            start_conditions,
        };

        assert!(test_continuous_pva(&input));
    }

    #[test]
    fn continuous_nonzero_v1_wrongdirection() {
        let constraints = SCurveConstraints {
            max_jerk: 3.,
            max_acceleration: 2.0,
            max_velocity: 3.,
        };
        let start_conditions = SCurveStartConditions {
            q0: 0.,
            q1: 10.,
            v0: 0.,
            v1: -1.,
        };
        let input = SCurveInput {
            constraints,
            start_conditions,
        };

        assert!(test_continuous_pva(&input));
    }

    #[test]
    fn continuous_nonzero_v0() {
        let constraints = SCurveConstraints {
            max_jerk: 3.,
            max_acceleration: 2.0,
            max_velocity: 3.,
        };
        let start_conditions = SCurveStartConditions {
            q0: 0.,
            q1: 10.,
            v0: 1.,
            v1: 0.,
        };
        let input = SCurveInput {
            constraints,
            start_conditions,
        };

        assert!(test_continuous_pva(&input));
    }

    #[test]
    fn continuous_nonzero_v0_wrongdir() {
        let constraints = SCurveConstraints {
            max_jerk: 3.,
            max_acceleration: 2.0,
            max_velocity: 3.,
        };
        let start_conditions = SCurveStartConditions {
            q0: 0.,
            q1: 10.,
            v0: 1.,
            v1: 0.,
        };
        let input = SCurveInput {
            constraints,
            start_conditions,
        };

        assert!(test_continuous_pva(&input));
    }

    #[test]
    fn continuous_negative() {
        let constraints = SCurveConstraints {
            max_jerk: 3.,
            max_acceleration: 2.0,
            max_velocity: 3.,
        };
        let start_conditions = SCurveStartConditions {
            q0: 10.,
            q1: 0.,
            v0: 0.,
            v1: 0.,
        };
        let input = SCurveInput {
            constraints,
            start_conditions,
        };

        assert!(test_continuous_pva(&input));
    }

    #[test]
    fn continuous_negative_nonzero_v0() {
        let constraints = SCurveConstraints {
            max_jerk: 3.,
            max_acceleration: 2.0,
            max_velocity: 3.,
        };
        let start_conditions = SCurveStartConditions {
            q0: 10.,
            q1: 0.,
            v0: -1.,
            v1: 0.,
        };
        let input = SCurveInput {
            constraints,
            start_conditions,
        };

        assert!(test_continuous_pva(&input));
    }

    #[test]
    fn continuous_negative_nonzero_v0_wrong_direction() {
        let constraints = SCurveConstraints {
            max_jerk: 3.,
            max_acceleration: 2.0,
            max_velocity: 3.,
        };
        let start_conditions = SCurveStartConditions {
            q0: 10.,
            q1: 0.,
            v0: 1.,
            v1: 0.,
        };
        let input = SCurveInput {
            constraints,
            start_conditions,
        };

        assert!(test_continuous_pva(&input));
    }
}

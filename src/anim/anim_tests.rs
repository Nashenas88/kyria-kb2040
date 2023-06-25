use super::*;
use assert_matches::assert_matches;

#[test]
fn test_anim() {
    let mut anim = AnimationController::new();
    assert!(!anim.tick(AnimKind::Boot.anim_time_ms() - 1));
    assert!(anim.tick(1));
    assert_matches!(
        anim.state,
        InternalState::Steady(AnimState {
            kind: AnimKind::Boot,
            anim_ms: 0,
        })
    );

    anim.set_state(AnimKind::Colemak);
    assert!(!anim.tick(AnimKind::Colemak.anim_time_ms() - 1));
    let leds = anim.leds();
    assert!(leds.len() > 0);
    assert_matches!(
        anim.state,
        InternalState::Steady(AnimState {
            kind: AnimKind::Colemak,
            anim_ms: 2999
        })
    );
    assert!(anim.tick(1));
    anim.set_state(AnimKind::Qwerty);
    assert!(!anim.tick(200));
    let leds = anim.leds();
    assert!(leds.len() > 0);
    assert_matches!(
        anim.state,
        InternalState::Transition(
            AnimState {
                kind: AnimKind::Colemak,
                anim_ms: 200
            },
            AnimState {
                kind: AnimKind::Qwerty,
                anim_ms: 200
            },
            201
        )
    );
    anim.tick(TRANSITION_MS - 201);
    let leds = anim.leds();
    assert!(leds.len() > 0);
    assert_matches!(
        anim.state,
        InternalState::Steady(AnimState {
            kind: AnimKind::Qwerty,
            anim_ms: 749
        })
    );
    anim.tick(10_000);
    let leds = anim.leds();
    assert!(leds.len() == 10);
    assert_matches!(
        anim.state,
        InternalState::Steady(AnimState {
            kind: AnimKind::Qwerty,
            anim_ms: 0
        })
    );

    // let mut counter = 0;
    // let mut custom_action = None;
    // let mut last = Instant::now();
    // loop {
    //     let now = Instant::now();
    //     let diff = now - last;
    //     last = now;
    //     let n: u64 = 250;
    //     std::thread::sleep(Duration::from_micros(
    //         n.saturating_sub(diff.as_micros() as u64),
    //     ));
    //     counter += 1;
    //     if counter >= 3000 / n {
    //         counter = 0;

    //         if let Some(anim_state) =
    //             custom_action
    //                 .take()
    //                 .and_then(|custom_action| match custom_action {
    //                     CustomAction::QwertyLed => Some(AnimState::Qwerty),
    //                     CustomAction::ColemakLed => Some(AnimState::Colemak),
    //                     CustomAction::LayerSelectLed => Some(AnimState::LayerSelect),
    //                     CustomAction::SymLed => Some(AnimState::Sym),
    //                     _ => None,
    //                 })
    //         {
    //             anim.set_state(anim_state);
    //         }

    //         // // Immediately clear the interrupt and schedule the next scan alarm.
    //         // a.clear_interrupt(t);
    //         // let _ = a.schedule(LED_ANIM_TIME_US.microseconds());

    //         // let mut ws = c.shared.ws;
    //         if anim.tick() {
    //             anim.set_state(AnimState::Colemak);
    //         }

    //         let leds = anim.leds();
    //         // SmartLedsWrite::write(&mut ws, brightness(leds.into_iter(), LED_BRIGHTNESS)).unwrap();
    //     }
    // }
}

#[test]
fn check_interp() {
    let rgb = AnimationController::interp(red(255), green(255), TRANSITION_MS * 2 / 3);
    assert_eq!(
        rgb,
        RGB {
            r: 85,
            g: 170,
            b: 0
        }
    );

    let anim = AnimationController::new();
    let leds = anim.leds_for_transition(
        &[red(255), green(255)],
        &[blue(255), blue(255)],
        TRANSITION_MS * 2 / 3,
    );
    assert_eq!(
        leds,
        [
            RGB {
                r: 85,
                g: 0,
                b: 170
            },
            RGB {
                r: 0,
                g: 85,
                b: 170
            }
        ]
    )
}

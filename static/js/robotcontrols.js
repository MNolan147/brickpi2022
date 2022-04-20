document.addEventListener("keydown", e => {
    keycode = e.code
    switch (keycode) {
        case "KeyW":
            new_ajax_helper("/moveforward")
        case "KeyS":
            new_ajax_helper("/movebackward")
        case "KeyA":
            new_ajax_helper("/turnleft")
        case "KeyD":
            new_ajax_helper("/turnright")
        case "Space":
            new_ajax_helper("/shoot")
    }
});

document.addEventListener("keyup", e => {
    new_ajax_helper("/stop")
});
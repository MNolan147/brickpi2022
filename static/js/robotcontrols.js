document.addEventListener("keydown", function(e){
    if (!e.repeat){
        key = e.key.toLowerCase();
        if (key === 'w') {
            new_ajax_helper("/moveforward")
        } else if (key === 's') {
            new_ajax_helper("/movebackward")
        } else if (key === 'a') {
            new_ajax_helper("/rotateleft")
        } else if (key === 'd') {
            new_ajax_helper("/rotateright")
        } else if (key === ' ') {
            new_ajax_helper("/shoot")
        }
    }
});

document.addEventListener("keyup", function(e){
    key = e.key.toLowerCase()
    if (key !== ' '){
        new_ajax_helper('/stop');
    }
});
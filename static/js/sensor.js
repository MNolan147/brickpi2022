const handle = setInterval(getSensorData, 1000);

function getSensorData() {
    new_ajax_helper('/sensors', receiveSensorData);
}

function receiveSensorData(results) {
    for (key in results) {
        document.getElementById(key).innerHTML = results[key];
    }
}
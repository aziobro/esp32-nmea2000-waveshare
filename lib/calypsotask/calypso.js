(function () {
    const api = window.esp32nmea2k;
    if (!api) return;
    api.registerListener((id, data) => {
        if (!data.calypso) return; //do nothing unless this board's task is active
        let page = api.addTabPage('calypso', 'Calypso Wind');
        api.addEl('div', 'hdg', page, 'Calypso Ultrasonic wind sensor (apparent wind, BLE)');
        function row(label) {
            let r = api.addEl('div', 'row', page);
            api.addEl('span', 'label', r, label);
            return api.addEl('span', 'value', r, '---');
        }
        let connVal = row('BLE connection');
        let speedVal = row('Apparent wind speed');
        let angleVal = row('Apparent wind angle');
        let battVal = row('Sensor battery');
        let tempVal = row('Air temperature');
        let attVal = row('Sensor roll/pitch');
        api.addEl('div', 'row', page,
            'No pairing needed - the sensor streams data as soon as it\'s in ' +
            'range and connected. Every Calypso Ultrasonic advertises the same ' +
            'BLE name, so this gateway locks onto the specific sensor it first ' +
            'connects to (see Config > calypso) rather than relying on the name ' +
            'alone. Roll/pitch/heading are the sensor\'s own tilt-compensation ' +
            'inputs, shown here for diagnostics only - not sent to NMEA2000.');
        function showDisconnected() {
            connVal.textContent = 'not connected';
            [speedVal, angleVal, battVal, tempVal, attVal].forEach((el) => el.textContent = 'no data');
        }
        function poll() {
            fetch('/api/user/calypsoTask/data')
                .then((res) => {
                    if (!res.ok) throw Error('server error: ' + res.status);
                    return res.json();
                })
                .then((json) => {
                    connVal.textContent = json.connected ? 'connected' : 'scanning...';
                    if (!json.connected || !json.valid) {
                        [speedVal, angleVal, battVal, tempVal, attVal].forEach((el) => el.textContent = 'no data');
                        return;
                    }
                    speedVal.textContent = (json.windSpeed * 1.94384).toFixed(1) + ' kn';
                    angleVal.textContent = json.windAngle.toFixed(0) + ' °';
                    battVal.textContent = json.battery + ' %';
                    tempVal.textContent = json.tempC.toFixed(1) + ' °C';
                    attVal.textContent = json.roll.toFixed(0) + ' ° / ' + json.pitch.toFixed(0) + ' °';
                })
                .catch((e) => {
                    console.log('calypso data fetch failed', e);
                    showDisconnected();
                });
        }
        poll();
        window.setInterval(poll, 1000);
    }, api.EVENTS.init);
})();

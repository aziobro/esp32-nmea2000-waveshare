(function () {
    const api = window.esp32nmea2k;
    if (!api) return;
    api.registerListener((id, data) => {
        if (!data.icm20948) return; //do nothing unless this board's task is active
        let page = api.addTabPage('icm20948', 'IMU');
        api.addEl('div', 'hdg', page, 'SparkFun ICM-20948 (heel/pitch/heading)');
        function row(label) {
            let r = api.addEl('div', 'row', page);
            api.addEl('span', 'label', r, label);
            return api.addEl('span', 'value', r, '---');
        }
        let fusionVal = row('Sensor fusion (DMP)');
        let rollVal = row('Roll (heel), calibrated');
        let pitchVal = row('Pitch (trim), calibrated');
        let rawRollVal = row('Roll, after invert (pre fine-offset)');
        let rawPitchVal = row('Pitch, after invert (pre fine-offset)');
        let hdgVal = row('Heading (magnetic), calibrated');
        let rotVal = row('Rate of turn');
        let accVal = row('Acceleration X/Y/Z (g)');
        api.addEl('div', 'row', page,
            'Sensor fusion and accel/gyro sensitivity/range can be set ' +
            'under Config > icm20948 (all take effect after Save/restart).');
        api.addEl('div', 'hdg', page, 'Roll/Pitch calibration');
        api.addEl('div', 'row', page,
            'With the boat confirmed level, go to Config > icm20948 and use ' +
            'the "C" button next to the Roll/Pitch calibration offset fields ' +
            '- it live-previews the offset that would zero out the current ' +
            'reading; type that number into the box and press Set. If a ' +
            'value has the wrong sign entirely (e.g. Roll goes negative ' +
            'while heeling to starboard), use the Invert toggle instead - ' +
            'see its description on the Config page.');
        api.addEl('div', 'hdg', page, 'Compass calibration');
        api.addEl('div', 'row', page,
            'Heading is OFF by default until calibrated (Config > icm20948 > ' +
            'Send Magnetic Heading). With sensor fusion ON (the default), the ' +
            'chip calibrates its own compass internally as part of the fused ' +
            'reading - the hard-iron X/Y offset fields below don\'t apply and ' +
            'can be left alone. With fusion OFF, this is a hard-iron-only ' +
            'calibration, not a full compass swing - press Reset on the Status ' +
            'page, then slowly rotate the WHOLE BOAT through at least one full ' +
            'circle (it\'s bolted down, it can\'t be waved around like a ' +
            'handheld sensor) while watching the "C" button preview for the ' +
            'X/Y offset fields stabilize, Set both. Either way, once heading ' +
            'looks reasonable, point the bow at a known heading and use the ' +
            'Heading offset field\'s "C" button the same way for the final ' +
            'fine calibration. Expect a few degrees of residual error.');
        function showInvalid() {
            [rollVal, pitchVal, rawRollVal, rawPitchVal, hdgVal, accVal].forEach((el) => el.textContent = 'no data');
        }
        function updateFusionRow(dmpActive) {
            fusionVal.textContent = dmpActive ? 'ON (accel+gyro+compass fused)' : 'OFF (plain accelerometer/compass math)';
        }
        function poll() {
            fetch('/api/user/icm20948Task/data')
                .then((res) => {
                    if (!res.ok) throw Error('server error: ' + res.status);
                    return res.json();
                })
                .then((json) => {
                    updateFusionRow(json.dmpActive);
                    // rate of turn is a raw gyro reading, computed every cycle
                    // regardless of whether the attitude/DMP sample below is
                    // ready yet, so update it unconditionally (even if the
                    // rest of the tab below shows "no data" briefly at startup).
                    rotVal.textContent = json.rot.toFixed(1) + '°/s';
                    if (!json.valid) {
                        showInvalid();
                        return;
                    }
                    rollVal.textContent = json.roll.toFixed(1) + '°';
                    pitchVal.textContent = json.pitch.toFixed(1) + '°';
                    rawRollVal.textContent = json.rawRoll.toFixed(1) + '°';
                    rawPitchVal.textContent = json.rawPitch.toFixed(1) + '°';
                    hdgVal.textContent = json.headingValid ? json.heading.toFixed(1) + '°' : 'no data';
                    accVal.textContent = json.accX.toFixed(2) + ' / ' + json.accY.toFixed(2) + ' / ' + json.accZ.toFixed(2) + ' g';
                })
                .catch((e) => {
                    console.log('icm20948 data fetch failed', e);
                    rotVal.textContent = 'no data';
                    showInvalid();
                });
        }
        poll();
        window.setInterval(poll, 1000);
    }, api.EVENTS.init);
})();

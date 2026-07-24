(function () {
    const api = window.esp32nmea2k;
    if (!api) return;
    api.registerListener((id, data) => {
        if (!data.icm20948) return; //do nothing unless this board's task is active
        let page = api.addTabPage('icm20948', 'IMU');
        api.addEl('div', 'hdg', page, 'SparkFun ICM-20948 (heel/pitch)');
        function row(label) {
            let r = api.addEl('div', 'row', page);
            api.addEl('span', 'label', r, label);
            return api.addEl('span', 'value', r, '---');
        }
        let rollVal = row('Roll (heel), calibrated');
        let pitchVal = row('Pitch (trim), calibrated');
        let rawRollVal = row('Roll, after invert (pre fine-offset)');
        let rawPitchVal = row('Pitch, after invert (pre fine-offset)');
        api.addEl('div', 'hdg', page, 'Calibration');
        api.addEl('div', 'row', page,
            'With the boat confirmed level, go to Config > icm20948 and use ' +
            'the "C" button next to the Roll/Pitch calibration offset fields ' +
            '- it live-previews the offset that would zero out the current ' +
            'reading; type that number into the box and press Set.');
        function showInvalid() {
            [rollVal, pitchVal, rawRollVal, rawPitchVal].forEach((el) => el.textContent = 'no data');
        }
        function poll() {
            fetch('/api/user/icm20948Task/data')
                .then((res) => {
                    if (!res.ok) throw Error('server error: ' + res.status);
                    return res.json();
                })
                .then((json) => {
                    if (!json.valid) {
                        showInvalid();
                        return;
                    }
                    rollVal.textContent = json.roll.toFixed(1) + '°';
                    pitchVal.textContent = json.pitch.toFixed(1) + '°';
                    rawRollVal.textContent = json.rawRoll.toFixed(1) + '°';
                    rawPitchVal.textContent = json.rawPitch.toFixed(1) + '°';
                })
                .catch((e) => {
                    console.log('icm20948 data fetch failed', e);
                    showInvalid();
                });
        }
        poll();
        window.setInterval(poll, 1000);
    }, api.EVENTS.init);
})();

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
        let hdgVal = row('Heading (output), calibrated');
        let rotVal = row('Rate of turn');
        let rotCheckVal = row('ROT vs heading-derivative cross-check');
        let accVal = row('Acceleration X/Y/Z (g)');
        api.addEl('div', 'row', page,
            'Sensor fusion and accel/gyro sensitivity/range can be set ' +
            'under Config > icm20948 (all take effect after Save/restart).');

        api.addEl('div', 'hdg', page, 'Heading source diagnostics');
        api.addEl('div', 'row', page,
            'The output heading above is chosen by "Heading source" (Config > ' +
            'icm20948) from up to three independently-computed candidates ' +
            'shown below: the chip\'s onboard DMP fusion, a software tilt-' +
            'compensated compass, and a software 9-axis (Mahony) fusion filter. ' +
            'DMP is validated (freshness, quaternion sanity, agreement with the ' +
            'compass) before use rather than trusted unconditionally - if it\'s ' +
            'rejected, "Rejection reasons" below says why.');
        let activeSourceVal = row('Active source');
        let qualityVal = row('Quality');
        let rejectionVal = row('Rejection reasons (if source is DMP)');
        let dmpHdgVal = row('DMP candidate heading');
        let compassHdgVal = row('Software compass candidate heading');
        let fusionHdgVal = row('Software fusion candidate heading');
        let magMagVal = row('Magnetic field magnitude');
        let magStateVal = row('Magnetic disturbance state');

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
            'Send Magnetic Heading). The hard-iron X/Y offset fields below ' +
            'feed the software compass candidate directly (unlike before, this ' +
            'now applies regardless of whether DMP/fusion is selected, since ' +
            'the compass always runs independently as a validation reference) ' +
            '- press Reset on the Status page, then slowly rotate the WHOLE ' +
            'BOAT through at least one full circle (it\'s bolted down, it ' +
            'can\'t be waved around like a handheld sensor) while watching the ' +
            '"C" button preview for the X/Y offset fields stabilize, Set both. ' +
            'Then point the bow at a known heading and use the Heading offset ' +
            'field\'s "C" button the same way for the final fine calibration. ' +
            'This is still a hard-iron-only fit (not a full compass swing) - ' +
            'expect a few degrees of residual, heading-dependent error; a full ' +
            '3D calibration workflow is planned but not yet available here.');

        function showInvalid() {
            [rollVal, pitchVal, rawRollVal, rawPitchVal, hdgVal, accVal].forEach((el) => el.textContent = 'no data');
        }
        function updateFusionRow(dmpActive) {
            fusionVal.textContent = dmpActive ? 'ON (chip DMP available as a candidate source)' : 'OFF (not initialized - only software compass/fusion available)';
        }

        const REJECT_FLAGS = [
            [1 << 0, 'initializing/converging'],
            [1 << 1, 'DMP sample stale'],
            [1 << 2, 'bad quaternion (NaN or bad norm)'],
            [1 << 3, 'magnetic field too low'],
            [1 << 4, 'magnetic field too high'],
            [1 << 5, 'magnetic field changed abruptly'],
            [1 << 6, 'disagrees with software compass'],
            [1 << 7, 'calibration invalid'],
            [1 << 8, 'sensor read error / DMP not active'],
            [1 << 9, 'FIFO error'],
            [1 << 10, 'sudden implausible jump'],
        ];
        function describeRejectionFlags(flags) {
            if (!flags) return 'none';
            let names = REJECT_FLAGS.filter(([bit]) => (flags & bit) !== 0).map(([, name]) => name);
            return names.length ? names.join(', ') : 'none';
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
                    rotCheckVal.textContent = json.rotDerived !== undefined
                        ? json.rotDerived.toFixed(1) + '°/s derived' + (json.rotDisagrees ? ' - DISAGREES' : ' (ok)')
                        : 'no data';

                    activeSourceVal.textContent = json.headingSource || 'none';
                    qualityVal.textContent = json.headingQuality || 'invalid';
                    rejectionVal.textContent = describeRejectionFlags(json.rejectionFlags);
                    dmpHdgVal.textContent = json.dmpHeadingValid ? json.dmpHeading.toFixed(1) + '°' : 'no data';
                    compassHdgVal.textContent = json.compassHeading !== undefined ? json.compassHeading.toFixed(1) + '°' : 'no data';
                    fusionHdgVal.textContent = json.fusionValid ? json.fusionHeading.toFixed(1) + '°' : 'converging...';
                    magMagVal.textContent = json.magMagnitude !== undefined ? json.magMagnitude.toFixed(1) : '---';
                    magStateVal.textContent = json.magDisturbed ? 'DISTURBED' : 'normal';

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

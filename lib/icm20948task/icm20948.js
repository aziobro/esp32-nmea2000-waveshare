(function () {
    const api = window.esp32nmea2k;
    if (!api) return;
    api.registerListener((id, data) => {
        if (!data.icm20948) return; //do nothing unless this board's task is active
        let page = api.addTabPage('icm20948', 'IMU');
        api.addEl('div', 'hdg', page, 'ICM-20948 IMU (heel/pitch/heading)');
        function row(label, parent) {
            let r = api.addEl('div', 'row', parent || page);
            api.addEl('span', 'label', r, label);
            return api.addEl('span', 'value', r, '---');
        }
        function actionButton(label, parent, onClick) {
            let btn = api.addEl('button', '', parent, label);
            btn.addEventListener('click', onClick);
            return btn;
        }
        function clearGyroSavedState() {
            gyroCalSaved = false;
        }
        function base() { return '/api/user/icm20948Task/'; }
        function callAction(endpoint, params) {
            let qs = params ? '?' + Object.keys(params).map((k) => encodeURIComponent(k) + '=' + encodeURIComponent(params[k])).join('&') : '';
            return fetch(base() + endpoint + qs).then((r) => r.json());
        }

        let fusionVal = row('Software fusion');
        let rollVal = row('Roll (heel), calibrated');
        let pitchVal = row('Pitch (trim), calibrated');
        let rawRollVal = row('Roll, after invert (pre fine-offset)');
        let rawPitchVal = row('Pitch, after invert (pre fine-offset)');
        let hdgVal = row('Heading (output), calibrated');
        let rotVal = row('Rate of turn');
        let rotCheckVal = row('ROT vs heading-derivative cross-check');
        let accVal = row('Acceleration X/Y/Z (g)');
        api.addEl('div', 'row', page,
            'Software fusion and accel/gyro sensitivity/range can be set ' +
            'under Config > icm20948 (all take effect after Save/restart).');

        api.addEl('div', 'hdg', page, 'Heading source diagnostics');
        api.addEl('div', 'row', page,
            'The output heading above is chosen by "Heading source" (Config > ' +
            'icm20948) from the software tilt-compensated compass and software ' +
            '9-axis (Mahony) fusion candidates shown below.');
        let activeSourceVal = row('Active source');
        let qualityVal = row('Quality');
        let rejectionVal = row('Rejection reasons');
        let compassHdgVal = row('Software compass candidate heading');
        let fusionHdgVal = row('Software fusion candidate heading');
        let disagreementVal = row('Candidate disagreement (max pairwise)');
        let magMagVal = row('Magnetic field magnitude');
        let magStateVal = row('Magnetic disturbance state');
        let holdoverVal = row('Holdover (gyro-only continuation)');

        api.addEl('div', 'hdg', page, 'Raw magnetic data');
        api.addEl('div', 'row', page,
            'Diagnostic view of the magnetometer at each pipeline stage - useful ' +
            'while checking mounting orientation or judging calibration quality ' +
            'without leaving this tab.');
        let magRawVal = row('Mag source raw X/Y/Z');
        let magAgmtRawVal = row('AGMT register raw X/Y/Z');
        let magBoatVal = row('Boat frame X/Y/Z (after orientation transform)');
        let magCorrVal = row('Corrected X/Y/Z (after calibration)');
        let magMagVal2 = row('Magnitude');
        let magStateVal2 = row('Disturbance state');
        let imuErrorsVal = row('IMU communication errors');
        let imuRecoveriesVal = row('IMU recoveries');
        let imuStatusVal = row('Last IMU status');

        api.addEl('div', 'hdg', page, 'Roll/Pitch calibration');
        api.addEl('div', 'row', page,
            'With the boat confirmed level, go to Config > icm20948 and use ' +
            'the "C" button next to the Roll/Pitch calibration offset fields ' +
            '- it live-previews the offset that would zero out the current ' +
            'reading; type that number into the box and press Set. If a ' +
            'value has the wrong sign entirely (e.g. Roll goes negative ' +
            'while heeling to starboard), use the Invert toggle instead - ' +
            'see its description on the Config page.');

        api.addEl('div', 'hdg', page, 'Calibration');
        api.addEl('div', 'row', page,
            'Export/Import/Reset exchange the full magnetometer+gyroscope ' +
            'calibration as JSON - use Export to save a backup or feed the ' +
            'offline calibration tool (tools/icm20948_calibration/) a reference, ' +
            'and Import to load a file it produced (validated on the device ' +
            'before anything is saved - a bad file changes nothing). The ' +
            '"2D boat swing" section below is a quick on-device alternative: ' +
            'Start, then slowly rotate the WHOLE BOAT through at least one full ' +
            'circle (it\'s bolted down, it can\'t be tumbled through 3D ' +
            'orientations the way a handheld sensor can - only heading/X-Y ' +
            'calibration is done this way), Stop, review the result below, ' +
            'then Save if it looks reasonable or Cancel to discard it. This is ' +
            'a simpler fit than the offline tool\'s full 3D ellipsoid - use the ' +
            'offline tool for the best result if you can remove the sensor.');

        let calJsonArea = api.addEl('textarea', 'calJson', page);
        calJsonArea.rows = 6;
        calJsonArea.style.width = '100%';
        calJsonArea.placeholder = 'Exported calibration JSON appears here - or paste a file to Import.';
        let calButtonsRow = api.addEl('div', 'row', page);
        actionButton('Export', calButtonsRow, () => {
            callAction('calControl', { action: 'export' }).then((json) => {
                calJsonArea.value = JSON.stringify(json, null, 2);
            }).catch((e) => alert('export failed: ' + e));
        });
        actionButton('Import', calButtonsRow, () => {
            let text = calJsonArea.value;
            if (!text || !text.trim()) { alert('paste calibration JSON into the box first'); return; }
            callAction('calControl', { action: 'import', json: text }).then((res) => {
                if (res.status === 'OK') alert('calibration imported');
                else alert('import rejected: ' + (res.message || 'unknown error'));
            }).catch((e) => alert('import failed: ' + e));
        });
        actionButton('Reset to identity', calButtonsRow, () => {
            if (!confirm('Reset calibration to identity (no bias, no correction)? This cannot be undone.')) return;
            callAction('calControl', { action: 'reset' }).then(() => alert('calibration reset')).catch((e) => alert('reset failed: ' + e));
        });

        api.addEl('div', 'hdg', page, '2D boat swing (magnetometer)');
        let magCalStateVal = row('State');
        let magCalSamplesVal = row('Samples collected');
        let magCalCoverageVal = row('Sector coverage');
        let magCalSpanVal = row('Swept arc');
        let magCalResultVal = row('Bias X/Y, scale X/Y');
        let magCalQualityVal = row('Quality');
        let magCalReasonVal = row('Save unavailable because');
        let magCalButtonsRow = api.addEl('div', 'row', page);
        actionButton('Start', magCalButtonsRow, () => callAction('magCalControl', { action: 'start' }).catch((e) => alert('start failed: ' + e)));
        actionButton('Stop', magCalButtonsRow, () => callAction('magCalControl', { action: 'stop' }).catch((e) => alert('stop failed: ' + e)));
        let magCalSaveBtn = actionButton('Save', magCalButtonsRow, () => {
            callAction('magCalControl', { action: 'save' }).then((res) => {
                if (res.status === 'error') alert('save failed: ' + res.message);
            }).catch((e) => alert('save failed: ' + e));
        });
        actionButton('Cancel', magCalButtonsRow, () => callAction('magCalControl', { action: 'cancel' }).catch((e) => alert('cancel failed: ' + e)));

        api.addEl('div', 'hdg', page, 'Gyroscope calibration');
        api.addEl('div', 'row', page,
            'Measures the gyroscope\'s stationary bias - keep the boat COMPLETELY ' +
            'STILL and level for the duration (a sample is rejected, without ' +
            'losing progress, if motion or tilt away from level is detected).');
        let gyroCalStateVal = row('State');
        let gyroCalProgressVal = row('Progress');
        let gyroCalResultVal = row('Bias X/Y/Z (deg/s)');
        let gyroCalStdDevVal = row('Std deviation X/Y/Z (deg/s)');
        let gyroCalButtonsRow = api.addEl('div', 'row', page);
        let gyroCalSaved = false;
        actionButton('Start', gyroCalButtonsRow, () => {
            clearGyroSavedState();
            callAction('gyroCalControl', { action: 'start' }).catch((e) => alert('start failed: ' + e));
        });
        let gyroCalSaveBtn = actionButton('Save', gyroCalButtonsRow, () => {
            callAction('gyroCalControl', { action: 'save' }).then((res) => {
                if (res.status === 'error') {
                    alert('save failed: ' + res.message);
                    return;
                }
                gyroCalSaved = true;
                gyroCalResultVal.textContent = fmtXYZ(res.biasX, res.biasY, res.biasZ, 3) + ' (saved)';
                gyroCalStdDevVal.textContent = fmtXYZ(res.stdDevX, res.stdDevY, res.stdDevZ, 3);
            }).catch((e) => alert('save failed: ' + e));
        });
        actionButton('Cancel', gyroCalButtonsRow, () => {
            clearGyroSavedState();
            callAction('gyroCalControl', { action: 'cancel' }).catch((e) => alert('cancel failed: ' + e));
        });

        api.addEl('div', 'hdg', page, 'Diagnostic logging');
        api.addEl('div', 'row', page,
            'Captures every sensor sample (raw/boat-frame/corrected values, all ' +
            'heading candidates, quality/rejection flags) as CSV for the offline ' +
            'calibration tool or general troubleshooting - see ' +
            'tools/icm20948_calibration/README.md.');
        let logStateVal = row('State');
        let logCapturedVal = row('Samples captured / written / dropped');
        let logBufferVal = row('Buffer used');
        let captureParams = () => ({
            rateHz: rateInput.value,
            maxDurationSec: durationInput.value,
            maxKB: maxKbInput.value
        });
        let logButtonsRow = api.addEl('div', 'row', page);
        actionButton('Start', logButtonsRow, () => callAction('capControl', Object.assign({ action: 'start' }, captureParams())).catch((e) => alert('start failed: ' + e)));
        actionButton('Stop', logButtonsRow, () => callAction('capControl', { action: 'stop' }).catch((e) => alert('stop failed: ' + e)));
        actionButton('Clear', logButtonsRow, () => callAction('capControl', { action: 'clear' }).catch((e) => alert('clear failed: ' + e)));
        actionButton('Download', logButtonsRow, () => { window.location = base() + 'capDownload'; });
        let serialToggleBtn = actionButton('Serial output: off', logButtonsRow, () => {
            let turningOn = serialToggleBtn.textContent.indexOf('off') >= 0;
            callAction('capControl', { action: turningOn ? 'serialOn' : 'serialOff' }).catch((e) => alert('toggle failed: ' + e));
        });
        let rateRow = api.addEl('div', 'row', page);
        api.addEl('span', 'label', rateRow, 'Capture rate (Hz)');
        let rateInput = api.addEl('input', '', rateRow);
        rateInput.type = 'number';
        rateInput.min = 1;
        rateInput.max = 100;
        rateInput.value = 10;
        api.addEl('span', 'label', rateRow, 'Duration (sec)');
        let durationInput = api.addEl('input', '', rateRow);
        durationInput.type = 'number';
        durationInput.min = 1;
        durationInput.max = 3600;
        durationInput.value = 180;
        api.addEl('span', 'label', rateRow, 'Max KB');
        let maxKbInput = api.addEl('input', '', rateRow);
        maxKbInput.type = 'number';
        maxKbInput.min = 1;
        maxKbInput.max = 512;
        maxKbInput.value = 512;
        actionButton('Apply', rateRow, () => callAction('capControl', captureParams()).catch((e) => alert('apply failed: ' + e)));

        api.addEl('div', 'hdg', page, 'Performance');
        api.addEl('div', 'row', page,
            'Per-cycle timing and resource counters - see doc/IcmPerformanceReview.md. ' +
            '"missed deadlines" counts cycles where total processing took longer than the ' +
            'configured update interval (Config > icm20948 > Update rate).');
        let perfLoopVal = row('Loop time (last / max)');
        let perfSensorVal = row('Sensor read (last / max)');
        let perfProcessingVal = row('Pipeline processing (last / max)');
        let perfFusionVal = row('Fusion filter (last / max)');
        let perfNmeaVal = row('NMEA send (last / max)');
        let perfLoggingVal = row('Logging enqueue (last / max)');
        let perfMissedVal = row('Missed deadlines');
        let perfSensorErrorsVal = row('Sensor errors / reinits');
        let perfHeapVal = row('Free heap / min-ever free heap');
        let perfStackVal = row('Task stack high-water mark');

        function showInvalid() {
            [rollVal, pitchVal, rawRollVal, rawPitchVal, hdgVal, accVal].forEach((el) => el.textContent = 'no data');
        }
        function updateFusionRow(json) {
            fusionVal.textContent = json.fusionValid ? 'valid' : 'converging';
        }

        const REJECT_FLAGS = [
            [1 << 0, 'initializing/converging'],
            [1 << 3, 'magnetic field too low'],
            [1 << 4, 'magnetic field too high'],
            [1 << 5, 'magnetic field changed abruptly'],
            [1 << 7, 'calibration invalid'],
            [1 << 8, 'sensor read error'],
            [1 << 11, 'fusion disagrees with software compass'],
            [1 << 12, 'magnetometer sample invalid'],
        ];
        function describeRejectionFlags(flags) {
            if (!flags) return 'none';
            let names = REJECT_FLAGS.filter(([bit]) => (flags & bit) !== 0).map(([, name]) => name);
            return names.length ? names.join(', ') : 'none';
        }
        function circularDiffDeg(a, b) {
            let d = Math.abs(a - b) % 360;
            return d > 180 ? 360 - d : d;
        }
        function describeDisagreement(json) {
            let candidates = [];
            if (json.compassHeading !== undefined) candidates.push(json.compassHeading);
            if (json.fusionValid) candidates.push(json.fusionHeading);
            if (candidates.length < 2) return 'n/a (fewer than 2 valid candidates)';
            let maxDiff = 0;
            for (let i = 0; i < candidates.length; i++)
                for (let j = i + 1; j < candidates.length; j++)
                    maxDiff = Math.max(maxDiff, circularDiffDeg(candidates[i], candidates[j]));
            return maxDiff.toFixed(1) + '°';
        }
        function fmtXYZ(x, y, z, digits) {
            return x.toFixed(digits) + ' / ' + y.toFixed(digits) + ' / ' + z.toFixed(digits);
        }

        function poll() {
            fetch('/api/user/icm20948Task/data')
                .then((res) => {
                    if (!res.ok) throw Error('server error: ' + res.status);
                    return res.json();
                })
                .then((json) => {
                    updateFusionRow(json);
                    // rate of turn is a raw gyro reading, computed every cycle
                    // regardless of whether the attitude sample below is
                    // ready yet, so update it unconditionally (even if the
                    // rest of the tab below shows "no data" briefly at startup).
                    rotVal.textContent = json.rot.toFixed(1) + '°/s';
                    rotCheckVal.textContent = json.rotDerived !== undefined
                        ? json.rotDerived.toFixed(1) + '°/s derived' + (json.rotDisagrees ? ' - DISAGREES' : ' (ok)')
                        : 'no data';

                    activeSourceVal.textContent = json.headingSource || 'none';
                    qualityVal.textContent = json.headingQuality || 'invalid';
                    rejectionVal.textContent = describeRejectionFlags(json.rejectionFlags);
                    compassHdgVal.textContent = json.compassHeading !== undefined ? json.compassHeading.toFixed(1) + '°' : 'no data';
                    fusionHdgVal.textContent = json.fusionValid ? json.fusionHeading.toFixed(1) + '°' : 'converging...';
                    disagreementVal.textContent = describeDisagreement(json);
                    magMagVal.textContent = json.magMagnitude !== undefined ? json.magMagnitude.toFixed(1) : '---';
                    magStateVal.textContent = json.magDisturbed ? 'DISTURBED' : 'normal';
                    holdoverVal.textContent = json.headingHoldover
                        ? 'ACTIVE - dead-reckoning from gyro, NOT sent as PGN 127250 (state: ' + json.headingHoldoverState + ')'
                        : 'not active (state: ' + (json.headingHoldoverState || 'unknown') + ')';

                    if (json.magRawX !== undefined) {
                        magRawVal.textContent = fmtXYZ(json.magRawX, json.magRawY, json.magRawZ, 1) +
                            ' (' + (json.magSource || 'unknown') + ', ' + (json.magValid ? 'valid' : 'invalid') + ')';
                        magAgmtRawVal.textContent = json.magAgmtRawX !== undefined
                            ? fmtXYZ(json.magAgmtRawX, json.magAgmtRawY, json.magAgmtRawZ, 1)
                            : '---';
                        magBoatVal.textContent = fmtXYZ(json.magBoatX, json.magBoatY, json.magBoatZ, 1);
                        magCorrVal.textContent = fmtXYZ(json.magCorrX, json.magCorrY, json.magCorrZ, 1);
                        magMagVal2.textContent = json.magMagnitude.toFixed(1);
                        magStateVal2.textContent = json.magDisturbed ? 'DISTURBED' : 'normal';
                    }
                    imuErrorsVal.textContent = 'sensor ' + (json.sensorErrorCount ?? 0) +
                        ', i2c retries ' + (json.i2cRetryCount ?? 0);
                    imuRecoveriesVal.textContent = 'sensor reinits ' + (json.sensorReinitCount ?? 0) +
                        ', i2c recoveries ' + (json.i2cRecoveryCount ?? 0) +
                        ', mag recoveries ' + (json.magRecoveryCount ?? 0);
                    imuStatusVal.textContent = json.lastIcmStatus || '---';

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

            callAction('magCalStatus').then((json) => {
                magCalStateVal.textContent = json.state;
                magCalSamplesVal.textContent = json.sampleCount;
                magCalCoverageVal.textContent = (json.coverageFraction * 100).toFixed(0) + '%';
                magCalSpanVal.textContent = json.spanDeg.toFixed(0) + '°';
                if (json.state === 'ready') {
                    magCalResultVal.textContent = json.biasX.toFixed(2) + ' / ' + json.biasY.toFixed(2) + ', ' +
                        json.scaleX.toFixed(3) + ' / ' + json.scaleY.toFixed(3);
                    magCalQualityVal.textContent = (json.quality * 100).toFixed(0) + '%';
                    magCalReasonVal.textContent = 'nothing - ready to save';
                    magCalSaveBtn.disabled = false;
                } else {
                    magCalResultVal.textContent = '---';
                    magCalQualityVal.textContent = '---';
                    magCalReasonVal.textContent = json.state === 'failed' ? (json.failureReason || 'unknown') : 'swing not completed yet (Start, swing, then Stop)';
                    magCalSaveBtn.disabled = true;
                }
            }).catch((e) => console.log('magCalStatus fetch failed', e));

            callAction('gyroCalStatus').then((json) => {
                gyroCalStateVal.textContent = json.state;
                gyroCalProgressVal.textContent = json.sampleCount + ' / ' + json.requiredSamples;
                if (json.state === 'done') {
                    gyroCalResultVal.textContent = fmtXYZ(json.biasX, json.biasY, json.biasZ, 3) + (gyroCalSaved ? ' (saved)' : '');
                    gyroCalStdDevVal.textContent = fmtXYZ(json.stdDevX, json.stdDevY, json.stdDevZ, 3);
                    gyroCalSaveBtn.disabled = false;
                } else {
                    gyroCalSaved = false;
                    gyroCalResultVal.textContent = '---';
                    gyroCalStdDevVal.textContent = '---';
                    gyroCalSaveBtn.disabled = true;
                }
            }).catch((e) => console.log('gyroCalStatus fetch failed', e));

            callAction('capStatus').then((json) => {
                logStateVal.textContent = json.active ? 'capturing' : 'idle';
                logCapturedVal.textContent = json.samplesCaptured + ' / ' + json.samplesWritten + ' / ' + json.samplesDropped;
                logBufferVal.textContent = json.bufferBytesUsed + ' / ' + json.bufferBytesCapacity + ' bytes';
                serialToggleBtn.textContent = 'Serial output: ' + (json.serialEnabled ? 'on' : 'off');
                rateInput.value = json.rateHz;
                durationInput.value = json.maxDurationSec;
                maxKbInput.value = json.maxKB;
            }).catch((e) => console.log('capStatus fetch failed', e));

            callAction('perfStatus').then((json) => {
                let us = (v) => v + ' us';
                perfLoopVal.textContent = us(json.totalLoopUs) + ' / ' + us(json.totalLoopMaxUs);
                perfSensorVal.textContent = us(json.sensorReadUs) + ' / ' + us(json.sensorReadMaxUs);
                perfProcessingVal.textContent = us(json.processingUs) + ' / ' + us(json.processingMaxUs);
                perfFusionVal.textContent = us(json.fusionUs) + ' / ' + us(json.fusionMaxUs);
                perfNmeaVal.textContent = us(json.nmeaSendUs) + ' / ' + us(json.nmeaSendMaxUs);
                perfLoggingVal.textContent = us(json.loggingEnqueueUs) + ' / ' + us(json.loggingEnqueueMaxUs);
                perfMissedVal.textContent = json.missedDeadlines;
                perfSensorErrorsVal.textContent = (json.sensorErrors ?? 0) + ' / ' + (json.sensorReinits ?? 0);
                perfHeapVal.textContent = json.freeHeapBytes + ' / ' + json.minFreeHeapEverBytes + ' bytes';
                perfStackVal.textContent = json.stackHighWaterMarkBytes + ' bytes';
            }).catch((e) => console.log('perfStatus fetch failed', e));
        }
        poll();
        window.setInterval(poll, 1000);
    }, api.EVENTS.init);
})();

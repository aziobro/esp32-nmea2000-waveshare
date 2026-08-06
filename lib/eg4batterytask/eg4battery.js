(function () {
    const api = window.esp32nmea2k;
    if (!api) return;
    api.registerListener((id, data) => {
        if (!data.eg4battery) return; //do nothing unless this board's task is active
        let page = api.addTabPage('eg4battery', 'EG4 Battery (RS485)');
        api.addEl('div', 'hdg', page, 'Passive RS485 Modbus sniffer - never transmits');
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
        function base() { return '/api/user/eg4BatteryTask/'; }
        function callAction(endpoint, params) {
            let qs = params ? '?' + Object.keys(params).map((k) => encodeURIComponent(k) + '=' + encodeURIComponent(params[k])).join('&') : '';
            return fetch(base() + endpoint + qs).then((r) => r.json());
        }

        api.addEl('div', 'row', page,
            'Phase 1: raw capture only. This listens in on an EG4 battery\'s ' +
            'existing Modbus RTU conversation with its inverter (tapped in ' +
            'parallel on the RS485 bus) and records the frames it sees - it does ' +
            'not decode battery values yet, since no register map is known. ' +
            'Link settings (baud/format/gap) are under Config > eg4battery and ' +
            'take effect live, no reflash needed.');

        let linkVal = row('Link');
        let bytesVal = row('Bytes seen');
        let framesVal = row('Frames seen / CRC-valid');
        let lastFrameVal = row('Last frame');

        let capHdr = api.addEl('div', 'hdg', page, 'Capture');
        let capActiveVal = row('Capturing', page);
        let capFramesVal = row('Frames captured', page);
        let capBufVal = row('Buffer used', page);

        let btnRow = api.addEl('div', 'row', page);
        actionButton('Start (5 min, 128KB)', btnRow, () =>
            callAction('capControl', { action: 'start', maxDurationSec: 300, maxKB: 128 })
                .catch((e) => alert('start failed: ' + e)));
        actionButton('Stop', btnRow, () => callAction('capControl', { action: 'stop' }).catch((e) => alert('stop failed: ' + e)));
        actionButton('Clear', btnRow, () => callAction('capControl', { action: 'clear' }).catch((e) => alert('clear failed: ' + e)));
        actionButton('Download', btnRow, () => { window.location = base() + 'capDownload'; });

        function poll() {
            fetch(base() + 'data')
                .then((res) => {
                    if (!res.ok) throw Error('server error: ' + res.status);
                    return res.json();
                })
                .then((json) => {
                    linkVal.textContent = json.baud + ' baud, ' + json.format;
                    bytesVal.textContent = json.bytesSeen;
                    framesVal.textContent = json.framesSeen + ' / ' + json.framesValid;
                    lastFrameVal.textContent = json.lastFrameAgoMs >= 0 ? (json.lastFrameAgoMs / 1000).toFixed(1) + 's ago' : 'none yet';
                })
                .catch((e) => console.log('eg4battery data fetch failed', e));
            callAction('capStatus')
                .then((json) => {
                    capActiveVal.textContent = json.active ? 'yes (' + (json.captureDurationMs / 1000).toFixed(0) + 's)' : 'no';
                    capFramesVal.textContent = json.framesCaptured;
                    capBufVal.textContent = json.bufferBytesUsed + ' / ' + json.bufferBytesCapacity + ' bytes';
                })
                .catch((e) => console.log('eg4battery capStatus fetch failed', e));
        }
        poll();
        window.setInterval(poll, 2000);
    }, api.EVENTS.init);
})();

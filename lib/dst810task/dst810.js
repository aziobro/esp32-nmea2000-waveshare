(function () {
    const api = window.esp32nmea2k;
    if (!api) return;
    api.registerListener((id, data) => {
        if (!data.dst810) return; //do nothing unless this board's task is active
        let page = api.addTabPage('dst810', 'DST810');
        api.addEl('div', 'hdg', page, 'Airmar/B&G DST810 (depth/speed/water temp, BLE)');
        function row(label) {
            let r = api.addEl('div', 'row', page);
            api.addEl('span', 'label', r, label);
            return api.addEl('span', 'value', r, '---');
        }
        let connVal = row('BLE connection');
        let depthVal = row('Depth below transducer');
        let offsetVal = row('Transducer offset');
        let speedVal = row('Speed through water');
        let tempVal = row('Water temperature');
        api.addEl('div', 'row', page,
            'No pairing needed - the sensor streams data as soon as it\'s in ' +
            'range and connected. Enable/disable and the BLE name filter are ' +
            'under Config > dst810. The transducer offset comes from the ' +
            'sensor\'s own configuration (set up via Airmar\'s app), not from ' +
            'this gateway.');
        function showDisconnected() {
            connVal.textContent = 'not connected';
            [depthVal, offsetVal, speedVal, tempVal].forEach((el) => el.textContent = 'no data');
        }
        function poll() {
            fetch('/api/user/dst810Task/data')
                .then((res) => {
                    if (!res.ok) throw Error('server error: ' + res.status);
                    return res.json();
                })
                .then((json) => {
                    connVal.textContent = json.connected ? 'connected' : 'scanning...';
                    if (!json.connected || !json.valid) {
                        [depthVal, offsetVal, speedVal, tempVal].forEach((el) => el.textContent = 'no data');
                        return;
                    }
                    depthVal.textContent = json.depth.toFixed(2) + ' m';
                    offsetVal.textContent = json.offset.toFixed(2) + ' m';
                    speedVal.textContent = (json.speed * 1.94384).toFixed(2) + ' kn';
                    tempVal.textContent = json.tempC.toFixed(1) + ' °C';
                })
                .catch((e) => {
                    console.log('dst810 data fetch failed', e);
                    showDisconnected();
                });
        }
        poll();
        window.setInterval(poll, 1000);
    }, api.EVENTS.init);
})();

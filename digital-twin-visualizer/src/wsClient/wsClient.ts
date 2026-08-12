import { jointInfosStore } from "../stores";

export default class WSClient {
    private url: string;
    private ws: WebSocket | null = null;
    private reconnectDelay = 1000;
    private stopped = false;

    constructor(url: string) {
        this.url = url;
        console.log('WSClient constructor', url);
    }

    start() {
        console.log('WSClient start');
        this.connect();
    }

    stop() {
        this.stopped = true;
        if (this.ws) { this.ws.close(); this.ws = null; }
        console.log("Stopping WS client");
    }

    private connect() {
        if (this.stopped) return;
        this.ws = new WebSocket(this.url);

        this.ws.onopen = () => { console.log('[WS] open'); this.reconnectDelay = 1000; };

        this.ws.onmessage = (ev) => {
            try {
                const msg = JSON.parse(ev.data);
                if (msg.type === 'jointUpdate' && Array.isArray(msg.data)) {
                    // 直接 set 新数组以触发 Svelte 更新
                    jointInfosStore.set(msg.data);
                }
            } catch (e) { console.warn('[WS] parse err', e); }
        };

        this.ws.onclose = () => {
            console.log('[WS] closed, will reconnect in', this.reconnectDelay);
            if (!this.stopped) {
                setTimeout(() => this.connect(), this.reconnectDelay);
                this.reconnectDelay = Math.min(30000, this.reconnectDelay * 1.5);
            }
        };

        this.ws.onerror = (err) => {
            console.error('[WS] error', err);
        };
    }
}
// CurrentWSClient.ts
export default class CurrentWSClient {
    private url: string;
    private ws: WebSocket | null = null;
    private reconnectDelay = 1000;
    private stopped = false;

    // 定义一个回调函数，用于把解析后的数据传给外部
    private onDataReceived: (armA: number[], armB: number[]) => void;

    constructor(url: string, onData: (armA: number[], armB: number[]) => void) {
        this.url = url;
        this.onDataReceived = onData;
        console.log('CurrentWSClient constructor', url);
    }

    start() {
        this.stopped = false;
        this.connect();
    }

    stop() {
        this.stopped = true;
        if (this.ws) {
            this.ws.close();
            this.ws = null;
        }
    }

    private connect() {
        if (this.stopped) return;
        this.ws = new WebSocket(this.url);

        if (this.ws) {
            console.log('[CurrentWS] Connecting to', this.url);
        } else {
            console.error('[CurrentWS] WebSocket creation failed');
        }

        this.ws.onopen = () => {
            console.log('[CurrentWS] open');
            this.reconnectDelay = 1000;
        };

        this.ws.onmessage = (ev) => {
            if (!ev.data) return;

            try {
                // 先检查字符串是否以 { 开始，以 } 结束（简单预校验）
                const rawData = ev.data.trim();
                if (!rawData.startsWith('{') || !rawData.endsWith('}')) {
                    console.warn('[CurrentWS] 收到不完整的 JSON 数据帧');
                    return;
                }

                const data = JSON.parse(rawData);
                if (data.type === 'CURRENT_DATA') {
                    // console.log('[CurrentWS] Received data:', data);
                    this.onDataReceived(data.armA, data.armB);
                }
            } catch (e) {
                // 捕获错误，防止 bundle.js 停止运行
                console.error('[CurrentWS] 解析失败，原始数据片段:', ev.data.substring(0, 50) + '...');
            }
        };

        this.ws.onclose = () => {
            if (!this.stopped) {
                console.log('[CurrentWS] retry in', this.reconnectDelay);
                setTimeout(() => this.connect(), this.reconnectDelay);
                this.reconnectDelay = Math.min(10000, this.reconnectDelay * 1.5);
            }
        };

        this.ws.onerror = (err) => console.error('[CurrentWS] error', err);
    }
}

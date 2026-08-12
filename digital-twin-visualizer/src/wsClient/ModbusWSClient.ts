// ModbusWSClient.ts
// Connects to the backend Modbus WebSocket server (port 8083),
// receives real-time PLC sensor data and forwards it via callback.

export interface ModbusData {
    angle: number;
    length: number;
    slewingAngle: number;
    height: number;
    actualRadius: number;
    ratedRadius: number;
    amplitudeRatio: number;
    actualWeight: number;
    ratedWeight: number;
    torqueRatio: number;
    workTime: number;
    workRadius: number;
    engineRpm: number;
    oilLevel: number;
    lockFlag: number;
    alarmBits: number;
    statusBits: number;
    timestamp: string;
}

export default class ModbusWSClient {
    private url: string;
    private ws: WebSocket | null = null;
    private reconnectDelay = 1000;
    private stopped = false;

    private onDataReceived: (data: ModbusData) => void;

    constructor(url: string, onData: (data: ModbusData) => void) {
        this.url = url;
        this.onDataReceived = onData;
        console.log('ModbusWSClient constructor', url);
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
            console.log('[ModbusWS] Connecting to', this.url);
        } else {
            console.error('[ModbusWS] WebSocket creation failed');
        }

        this.ws.onopen = () => {
            console.log('[ModbusWS] open');
            this.reconnectDelay = 1000;
        };

        this.ws.onmessage = (ev) => {
            if (!ev.data) return;

            try {
                const rawData = ev.data.trim();
                if (!rawData.startsWith('{') || !rawData.endsWith('}')) {
                    console.warn('[ModbusWS] 收到不完整的 JSON 数据帧');
                    return;
                }

                const data = JSON.parse(rawData);
                if (data.type === 'MODBUS_DATA') {
                    this.onDataReceived(data as ModbusData);
                }
            } catch (e) {
                console.error('[ModbusWS] 解析失败，原始数据片段:', ev.data.substring(0, 50) + '...');
            }
        };

        this.ws.onclose = () => {
            if (!this.stopped) {
                console.log('[ModbusWS] retry in', this.reconnectDelay);
                setTimeout(() => this.connect(), this.reconnectDelay);
                this.reconnectDelay = Math.min(10000, this.reconnectDelay * 1.5);
            }
        };

        this.ws.onerror = (err) => console.error('[ModbusWS] error', err);
    }
}

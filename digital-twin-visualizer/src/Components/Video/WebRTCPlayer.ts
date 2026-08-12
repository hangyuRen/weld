/*
 * @Author: hyuRen
 * @Date: 2026-02-05 17:08:45
 * @LastEditors: hyuRen
 * @LastEditTime: 2026-02-05 17:53:05
 */
export async function startWebRTC(videoEl: HTMLVideoElement, stream: string) {

    const pc = new RTCPeerConnection({
        iceServers: []
    });

    pc.ontrack = (event) => {
        videoEl.srcObject = event.streams[0];
    };

    // 创建 SDP offer
    const offer = await pc.createOffer({
        offerToReceiveVideo: true,
        offerToReceiveAudio: false
    });

    await pc.setLocalDescription(offer);

    // 发送到 MediaMTX
    const res = await fetch(`http://localhost:8889/${stream}/whep`, {
        method: "POST",
        headers: {
            "Content-Type": "application/sdp"
        },
        body: offer.sdp
    });

    const answerSDP = await res.text();

    await pc.setRemoteDescription({
        type: "answer",
        sdp: answerSDP
    });
}


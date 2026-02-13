const statusEl = document.getElementById('status');
const videoEl = document.getElementById('video');

const wsUrl = `${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`;
const ws = new WebSocket(wsUrl);

const pc = new RTCPeerConnection({
  iceServers: [],
  sdpSemantics: 'unified-plan'
});

pc.ontrack = (event) => {
  if (videoEl.srcObject !== event.streams[0]) {
    videoEl.srcObject = event.streams[0];
    statusEl.textContent = 'Flux reçu';
  }
};

pc.onconnectionstatechange = () => {
  statusEl.textContent = `État: ${pc.connectionState}`;
};

pc.oniceconnectionstatechange = () => {
  statusEl.textContent = `ICE: ${pc.iceConnectionState}`;
};

ws.onopen = async () => {
  statusEl.textContent = 'Signalisation...';
  const offer = await pc.createOffer({
    offerToReceiveVideo: true,
    offerToReceiveAudio: false
  });
  await pc.setLocalDescription(offer);
  ws.send(JSON.stringify({ type: 'offer', sdp: offer.sdp }));
};

ws.onmessage = async (event) => {
  const data = JSON.parse(event.data);
  if (data.type === 'answer') {
    await pc.setRemoteDescription({ type: 'answer', sdp: data.sdp });
    statusEl.textContent = 'Réponse reçue, connexion...';
  } else if (data.type === 'error') {
    statusEl.textContent = `Erreur: ${data.message}`;
  }
};

ws.onerror = () => {
  statusEl.textContent = 'Erreur WebSocket';
};

ws.onclose = () => {
  statusEl.textContent = 'WebSocket fermé';
};

import React, { useEffect, useRef, useState } from 'react';

const VideoStream = () => {
  const [isConnected, setIsConnected] = useState(false);
  const imgRef = useRef(null);

  useEffect(() => {
    const img = new Image();
    
    // Gère la connexion
    img.onload = () => {
      setIsConnected(true);
    };

    img.onerror = () => {
      setIsConnected(false);
      setTimeout(() => {
        // Retry avec timestamp pour forcer le rechargement
        img.src = `http://localhost:8081?t=${Date.now()}`;
      }, 1000);
    };

    // Lancer le chargement initial
    img.src = `http://localhost:8081`;
    
    if (imgRef.current) {
      imgRef.current.src = img.src;
    }

    return () => {
      img.onload = null;
      img.onerror = null;
    };
  }, []);

  return (
    <div style={{ textAlign: 'center', padding: '20px' }}>
      <h2>📹 Flux Vidéo Robot</h2>
      
      <div style={{
        position: 'relative',
        maxWidth: '800px',
        margin: '0 auto',
        backgroundColor: '#000',
        borderRadius: '8px',
        overflow: 'hidden'
      }}>
        <img 
          ref={imgRef}
          alt="Stream RTSP" 
          style={{ 
            width: '100%',
            height: 'auto',
            display: 'block'
          }}
        />
        
        <div style={{
          position: 'absolute',
          top: '10px',
          right: '10px',
          padding: '8px 12px',
          backgroundColor: isConnected ? '#4CAF50' : '#f44336',
          color: 'white',
          borderRadius: '4px',
          fontSize: '12px',
          fontWeight: 'bold'
        }}>
          {isConnected ? '🟢 Connecté' : '🔴 Déconnecté'}
        </div>
      </div>

      <p style={{ marginTop: '10px', color: '#666' }}>
        Latence: ~300-400ms | Qualité: 1500 kbps
      </p>
    </div>
  );
};

export default VideoStream;

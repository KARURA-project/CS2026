# --- RTSP CONFIG ---

#RTSP_HOST = "192.168.150.10"   # rover IP or hostname
RTSP_HOST = "127.0.0.1" 
RTSP_PORT = 8554

#RTSP_FRONT = f"rtsp://{RTSP_HOST}:{RTSP_PORT}/cam/front"
RTSP_FRONT = f"rtsp://{RTSP_HOST}:{RTSP_PORT}/test"
RTSP_REAR  = f"rtsp://{RTSP_HOST}:{RTSP_PORT}/cam/rear"
RTSP_ARM   = f"rtsp://{RTSP_HOST}:{RTSP_PORT}/cam/arm"
RTSP_SIDE  = f"rtsp://{RTSP_HOST}:{RTSP_PORT}/cam/side"
RTPS_OVERHEAD = f"rtsp://{RTSP_HOST}:{RTSP_PORT}/cam/overhead"

# Default stream
RTSP_URL = RTSP_FRONT

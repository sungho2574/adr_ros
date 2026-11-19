# adr_camera

`csi_camera` opens the Jetson IMX219 through Argus and publishes BGR frames on `/adr/camera/image_raw` and intrinsics on `/adr/camera/camera_info`. Standalone capture defaults to mode 3 at 1640×1232, 30 fps. The racing bringup uses this mode and downsamples to 320×240 in `nvvidconv` with bilinear interpolation before publishing.

The camera timestamps frames from GStreamer buffer PTS on the system clock when available, with arrival timestamps as a fallback. Argus must have exclusive use of the CSI camera. Stop camera previews before starting the racing bringup.

`image_relay` encodes or decodes JPEG streams and preserves the source image header.

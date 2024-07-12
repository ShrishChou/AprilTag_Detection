import qrcode
import io

def create_qr_code(object_type, focus_mm):
    # Combine information into a string
    data = f"{object_type}:{focus_mm}"
    
    # Generate QR code
    qr = qrcode.QRCode(version=1, box_size=10, border=5)
    qr.add_data(data)
    qr.make(fit=True)
    
    # Create an image from the QR code
    img = qr.make_image(fill_color="black", back_color="white")
    
    # Convert to bytes
    img_byte_arr = io.BytesIO()
    img.save(img_byte_arr)
    img_byte_arr = img_byte_arr.getvalue()
    
    return img_byte_arr

# Example usage
qr_image = create_qr_code("Mirror", 50)
with open("mirror_qr.png", "wb") as f:
    f.write(qr_image)

qr_image = create_qr_code("Lens", 30)
with open("lens_qr.png", "wb") as f:
    f.write(qr_image)
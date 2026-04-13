import torch
import torch.nn as nn
from torchvision import models, transforms
from PIL import Image
import os

model_path = "/home/sierra-95/Documents/potato_disease_detection/models/model_v2.pth"
image_path = "/home/sierra-95/Documents/potato_disease_detection/potato_disease_detection/images/early.png"

classes = ["Early_blight","Healthy","Late_blight"]

# ----- Image Transform -----
transform = transforms.Compose([
    transforms.Resize((224, 224)),
    transforms.ToTensor(),
    transforms.Normalize([0.485, 0.456, 0.406],
                         [0.229, 0.224, 0.225])
])

# ----- Load Model -----
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model = models.resnet18(weights=None)            
model.fc = nn.Linear(model.fc.in_features, len(classes))
model.load_state_dict(torch.load(model_path, map_location=device))
model.eval()

img = Image.open(image_path).convert("RGB")
img_t = transform(img).unsqueeze(0).to(device)

with torch.no_grad():
    outputs = model(img_t)
    probs = torch.softmax(outputs, dim=1)
    pred_idx = torch.argmax(probs, dim=1).item()

print(f"Predicted Class: {classes[pred_idx]}")
print(f"Probabilities: {probs.cpu().numpy()}")

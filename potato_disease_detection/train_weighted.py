import torch
import torch.nn as nn
from torch.utils.data import DataLoader, WeightedRandomSampler
from torchvision import datasets, transforms, models
import numpy as np

train_dir = "dataset/train"
val_dir   = "dataset/val"
num_classes = 3  
batch_size = 32
epochs = 20  
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")


train_transform = transforms.Compose([
    transforms.Resize((224, 224)),
    transforms.RandomHorizontalFlip(),
    transforms.RandomRotation(10),
    transforms.ColorJitter(brightness=0.2, contrast=0.2),
    transforms.ToTensor(),
    transforms.Normalize([0.485, 0.456, 0.406],
                         [0.229, 0.224, 0.225])
])

val_transform = transforms.Compose([
    transforms.Resize((224, 224)),
    transforms.ToTensor(),
    transforms.Normalize([0.485, 0.456, 0.406],
                         [0.229, 0.224, 0.225])
])

# ----- Datasets -----
train_ds = datasets.ImageFolder(train_dir, transform=train_transform)
val_ds   = datasets.ImageFolder(val_dir, transform=val_transform)

# ----- Weighted sampler to balance classes -----
# Count samples per class
class_counts = [0] * num_classes
for _, label in train_ds.samples:
    class_counts[label] += 1

# Inverse frequency weights
class_weights = 1. / torch.tensor(class_counts, dtype=torch.float)
samples_weight = [class_weights[label] for _, label in train_ds.samples]

sampler = WeightedRandomSampler(samples_weight, num_samples=len(samples_weight), replacement=True)

# ----- DataLoaders -----
train_loader = DataLoader(train_ds, batch_size=batch_size, sampler=sampler)
val_loader   = DataLoader(val_ds, batch_size=batch_size, shuffle=False)

# ----- Model -----
model = models.resnet18(weights=models.ResNet18_Weights.DEFAULT)
model.fc = nn.Linear(model.fc.in_features, num_classes)
model = model.to(device)

# ----- Loss & Optimizer -----
criterion = nn.CrossEntropyLoss()
optimizer = torch.optim.Adam(model.parameters(), lr=1e-4)

# ----- Training Loop -----
for epoch in range(epochs):
    model.train()
    total_loss = 0
    for images, labels in train_loader:
        images, labels = images.to(device), labels.to(device)
        optimizer.zero_grad()
        outputs = model(images)
        loss = criterion(outputs, labels)
        loss.backward()
        optimizer.step()
        total_loss += loss.item()
    print(f"Epoch [{epoch+1}/{epochs}], Loss: {total_loss/len(train_loader):.4f}")

# ----- Save Model -----
torch.save(model.state_dict(), "models/model_v2.pth")
print("model_v2.pth saved")

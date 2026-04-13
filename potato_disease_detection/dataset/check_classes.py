from torchvision import datasets, transforms

train_dir = "/home/sierra-95/Documents/potato_disease_detection/dataset/train"

# Dummy transform just to initialize ImageFolder
transform = transforms.Compose([transforms.ToTensor()])
train_ds = datasets.ImageFolder(train_dir, transform=transform)
for idx, cls in enumerate(train_ds.classes):
    print(f"{idx} -> {cls}")

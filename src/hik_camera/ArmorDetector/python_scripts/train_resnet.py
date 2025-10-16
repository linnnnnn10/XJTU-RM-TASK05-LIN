import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, Dataset
from torchvision import transforms, models
import cv2
import os
import numpy as np
from sklearn.metrics import accuracy_score
from tqdm import tqdm

class DigitDataset(Dataset):
    def __init__(self, data_dir, transform=None):
        self.data_dir = data_dir
        self.transform = transform
        # 增加负样本类别（第6类）
        self.classes = ['1', '2', '3', '4', '5', 'negative']
        self.samples = []
        
        for class_idx, class_name in enumerate(self.classes):
            class_dir = os.path.join(data_dir, class_name)
            if not os.path.exists(class_dir):
                print(f"警告: 类别目录 {class_dir} 不存在")
                continue
                
            for img_name in os.listdir(class_dir):
                if img_name.lower().endswith(('.png', '.jpg', '.jpeg')):
                    self.samples.append({
                        'image_path': os.path.join(class_dir, img_name),
                        'label': class_idx
                    })
        
        print(f"数据集加载完成: 总样本数 {len(self.samples)}")
        for i, class_name in enumerate(self.classes):
            class_count = sum(1 for sample in self.samples if sample['label'] == i)
            print(f"  {class_name}: {class_count} 个样本")
    
    def __len__(self):
        return len(self.samples)
    
    def __getitem__(self, idx):
        sample = self.samples[idx]
        image = cv2.imread(sample['image_path'])
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        if self.transform:
            image = self.transform(image)
            
        return image, sample['label']

def train_model():
    # 数据预处理
    transform = transforms.Compose([
        transforms.ToPILImage(),
        transforms.Resize((224, 224)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    ])
    
    # 📁📁 数据集导入入口：设置数据集路径
    data_dir = "/home/linnnnnn/ArmorDetector/data/dataset"  # 修改为实际数据集路径
    
    # 检查负样本目录是否存在
    negative_dir = os.path.join(data_dir, 'negative')
    if not os.path.exists(negative_dir):
        print(f"警告: 负样本目录 {negative_dir} 不存在，请确保有负样本数据")
    
    dataset = DigitDataset(data_dir, transform=transform)
    
    # 分割训练集和验证集
    train_size = int(0.8 * len(dataset))
    val_size = len(dataset) - train_size
    train_dataset, val_dataset = torch.utils.data.random_split(dataset, [train_size, val_size])
    
    train_loader = DataLoader(train_dataset, batch_size=32, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=32, shuffle=False)
    
    # 加载预训练的ResNet模型，输出层改为6类（5个数字+1个负样本）
    model = models.resnet18(pretrained=True)
    model.fc = nn.Linear(model.fc.in_features, 6)  # 6个类别（5个数字+负样本）
    
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model = model.to(device)
    
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=0.001)
    
    # 训练循环
    num_epochs = 50
    best_accuracy = 0
    
    # 使用tqdm包装epoch循环
    epoch_pbar = tqdm(range(num_epochs), desc="总训练进度", position=0, ncols=100)
    
    for epoch in epoch_pbar:
        model.train()
        running_loss = 0.0
        
        # 训练阶段
        train_pbar = tqdm(train_loader, desc=f'Epoch {epoch+1}/{num_epochs} [训练]', 
                         leave=False, ncols=100, position=1)
        
        for images, labels in train_pbar:
            images, labels = images.to(device), labels.to(device)
            
            optimizer.zero_grad()
            outputs = model(images)
            loss = criterion(outputs, labels)
            loss.backward()
            optimizer.step()
            
            running_loss += loss.item()
            train_pbar.set_postfix({'Loss': f'{loss.item():.4f}'})
        
        avg_loss = running_loss / len(train_loader)
        
        # 验证阶段
        model.eval()
        val_predictions = []
        val_labels = []
        
        val_pbar = tqdm(val_loader, desc=f'Epoch {epoch+1}/{num_epochs} [验证]', 
                       leave=False, ncols=100, position=1)
        
        with torch.no_grad():
            for images, labels in val_pbar:
                images, labels = images.to(device), labels.to(device)
                outputs = model(images)
                _, predicted = torch.max(outputs, 1)
                
                val_predictions.extend(predicted.cpu().numpy())
                val_labels.extend(labels.cpu().numpy())
        
        accuracy = accuracy_score(val_labels, val_predictions)
        
        # 更新总进度条
        epoch_pbar.set_postfix({
            '平均损失': f'{avg_loss:.4f}',
            '准确率': f'{accuracy:.4f}',
            '最佳准确率': f'{best_accuracy:.4f}'
        })
        
        # 保存最佳模型
        if accuracy > best_accuracy:
            best_accuracy = accuracy
            torch.save(model.state_dict(), 'best_resnet_model_with_negative.pth')
            print(f'✨ 新的最佳模型已保存! 准确率: {accuracy:.4f}')
    
    print(f'训练完成! 最佳准确率: {best_accuracy:.4f}')
    print('模型现在可以识别: 1, 2, 3, 4, 5 和 负样本(非数字)')

if __name__ == '__main__':
    train_model()
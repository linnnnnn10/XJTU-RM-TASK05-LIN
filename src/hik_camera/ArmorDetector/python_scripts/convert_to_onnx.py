import torch
import torch.nn as nn
from torchvision import models
import onnx
import os

def convert_to_onnx():
    # 获取当前脚本所在的目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 修改为正确的模型文件名（与训练代码保存的文件名一致）
    model_path = os.path.join(script_dir, 'best_resnet_model_with_negative.pth')
    
    # 检查模型文件是否存在
    if not os.path.exists(model_path):
        print(f"错误: 找不到模型文件: {model_path}")
        return
    
    # 加载训练好的模型
    model = models.resnet18(weights=None)  # 使用weights参数替代pretrained
    
    # 明确设置为6个类别（与训练代码一致）
    num_classes = 6  # 5个数字 + 负样本
    model.fc = nn.Linear(model.fc.in_features, num_classes)
    
    print(f"设置模型为 {num_classes} 个输出类别")
    print("类别分布: 数字1, 数字2, 数字3, 数字4, 数字5, 负样本")
    
    # 加载训练权重
    print(f"正在加载模型: {model_path}")
    try:
        checkpoint = torch.load(model_path, map_location='cpu')
        model.load_state_dict(checkpoint)
        print("模型权重加载成功")
    except Exception as e:
        print(f"加载模型权重时出错: {e}")
        print("请检查模型文件是否与训练代码生成的模型一致")
        return
    
    model.eval()
    
    # 创建示例输入
    dummy_input = torch.randn(1, 3, 224, 224)
    
    # 导出为ONNX格式 - 保存到脚本同一目录
    onnx_path = os.path.join(script_dir, "resnet_model.onnx")
    
    # 确保输出包含6个类别
    torch.onnx.export(model, dummy_input, onnx_path,
                     input_names=['input'],
                     output_names=['output'],
                     dynamic_axes={
                         'input': {0: 'batch_size'},
                         'output': {0: 'batch_size'}
                     },
                     opset_version=11,
                     export_params=True,
                     do_constant_folding=True)
    
    # 验证ONNX模型
    try:
        onnx_model = onnx.load(onnx_path)
        onnx.checker.check_model(onnx_model)
        
        # 检查输出维度
        output_shape = None
        for output in onnx_model.graph.output:
            if output.name == 'output':
                dims = output.type.tensor_type.shape.dim
                output_shape = [dim.dim_value for dim in dims]
                break
        
        if output_shape and output_shape[1] == 6:
            print(f"✓ ONNX模型验证成功！输出维度: {output_shape}")
            print("✓ 模型现在可以输出6个类别的概率:")
            print("  索引0: 数字1")
            print("  索引1: 数字2") 
            print("  索引2: 数字3")
            print("  索引3: 数字4")
            print("  索引4: 数字5")
            print("  索引5: 负样本")
        else:
            print(f"✗ ONNX模型输出维度不正确: {output_shape}，期望: [batch_size, 6]")
            
    except Exception as e:
        print(f"ONNX模型验证失败: {e}")
    
    print(f"模型已转换为ONNX格式并保存为: {onnx_path}")

if __name__ == '__main__':
    convert_to_onnx()
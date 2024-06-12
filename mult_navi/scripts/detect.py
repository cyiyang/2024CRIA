import torch
import torchvision.transforms as transforms
from PIL import Image
from mnist import Net
# 加载模型
model = Net()
model = torch.load('./model_Mnist.pth')
model.eval()
# 将图像转换为张量并进行归一化
for i in range(10):
    image = Image.open('pic_seg/Contours' + str(i) + '.png')#.convert('L')
    transform = transforms.Compose([
        transforms.Resize((28, 28)),#28
        transforms.ToTensor(),
        transforms.Normalize((0.1307,), (0.3081,))
    ])
    tensor = transform(image)
    # pil_image = transforms.ToPILImage()(tensor)
    # pil_image.save('transformed_image' + str(i) + '.png')

    # 对图像进行预测

    with torch.no_grad():
        output = model(tensor.unsqueeze(0))
    prediction = output.argmax(dim=1, keepdim=True)
    print('Prediction:', prediction.item())

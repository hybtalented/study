# 验证码识别算法
验证码识别主要算法分为两个部分：

1. [图像分割算法](../basic_theory/image/segmentation.md)
将图像分割成一个个只有一个文本的图片。参考文献： [<sup>1</sup>](#refer-cut-1)
2. [图像识别算法](../basic_theory/image/text_recognize.md)
对当个文本图片进行识别得到文本。

## 流程图
``` mermaid
flowchart TD
    subgraph 整体流程
       started((开始))
       get_roc[裁剪感兴趣区域]
       remove_disturb_line(干扰线去除)
       image_segmentaiton(字符分割)
       recognize(图像文本识别)
       ended((结束))  
    end
    subgraph 字符分割
       
    end
    image_segmentaiton ==> 字符分割
    started --> get_roc --> remove_disturb_line --> image_segmentaiton --> recognize --> ended

```
## 算法
### 裁剪感兴趣区域(ROC)
### 干扰线去除
``` java
private static BufferedImage removeLine(BufferedImage img, int px) {
        if (img != null) {
            int width = img.getWidth();
            int height = img.getHeight();

            for (int x = 0; x < width; x++) {
                List<Integer> list = new ArrayList<Integer>();
                for (int y = 0; y < height; y++) {
                    int count = 0;
                    while (y < height - 1 && isBlack(img.getRGB(x, y))) {
                        count++;
                        y++;
                    }
                    if (count <= px && count > 0) {
                        for (int i = 0; i <= count; i++) {
                            list.add(y - i);
                        }
                    }
                }
                if (list.size() != 0) {
                    for (int i = 0; i < list.size(); i++) {
                        img.setRGB(x, list.get(i), Color.white.getRGB());
                    }
                }
            }
        }
        return img;

    }

    public static boolean isBlack(int rgb){
    Color c = new Color(rgb);
    int b = c.getBlue();
    int r = c.getRed();
    int g = c.getGreen();
    int sum = r+g+b
    if(sum<10){
        return true;
    }
    return false;
    //sum的值越小（最小为零，黑色）颜色越重，
    //sum的值越大（最大值是225*3）颜色越浅，
    //sum的值小于10就算是黑色了.
    }
```
### 基于knn的验证码识别算法

<style>
.mume .node,.label {
    font-size: 13px;
}
</style>

## 参考

<div id="refer-cut-1"></div>

- [1] [两种文本类型验证码字符分割提取方法](http://xueshu.baidu.com/)


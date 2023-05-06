---
sidebar_position: 1
title: RoboticView
---

> RoboticView是使用Vue2开发的前端机械臂控制UI

## 功能

允许用户通过鼠标或者触摸的方式拖动机械臂，通过监听相应的事件，可以获得返还角度

## 使用方法
### 安装
```shell
npm i robotic-view --S
```
### Use in Vue
```js
<template>
  <div class="demo">
    <robotic-arm-view />
    <robotic-palm-view />
  </div>
</template>

<script>
import { RoboticArmView, RoboticPalmView } from "robotic-view"
export default {
  name: 'robotic-view-demo',
  components: {RoboticArmView, RoboticPalmView }
}
</script>

<style scoped>
.demo{
  display: grid;
  grid-template-columns: 50% 50%;
}
</style>

```

### Open Attributes
以下是开放出来的属性

#### RoboticArmView

| Parameters   | Description                                               | Type    | Available   | Default    |
| ------------ | --------------------------------------------------------- | ------- | ----------- | ---------- |
| showBorder   | 显示控件的边界，设置这个参数可以看到UI画面的宽高位置                                        | Boolean | -           | false      |
| canvasWidth  | 设置画板的宽度                                         | Number  | -           | 300        |
| canvasHeight | 设置画板的高度                                        | Number  | -           | 350        |
| baseColor    | 设置机械臂基座的颜色                                        | String  |             | black      |
| jonitColor   | 设置关节颜色                                           | String  |             | brown      |
| leverColor   | 设置连杆颜色                                           | String  |             | black      |
| endColor     | 设置末端端点颜色                                        | String  |             | aqua       |
| jointRadius  | 设置关节的圆形半径                                          | Number  |             | 10         |
| baseRadius   | 设置基座的半径                                          | Number  |             | 40         |
| leverWidth   | 设置连杆宽度                                          | Number  |             | 15         |
| isRosArm     | 是否是ROS机械臂，如果是的话，那么相应的返还角度则不限制在180° | Boolean |             | false      |
| withLabel    | 是否显示角度文字                            | Boolean |             | false      |
| textColor    | 设置显示文字的颜色                                | String  |             | \#3CB371   |
| fontStyle    | 设置文字的字体大小风格                                 | String  |             | 15px Arial |
| ikSolution   | 逆运动学算法类型              | String  | left, right | right      |
| xTranslation   | 控件整体向X轴偏移量             | Number  |  | 0      |
| yTranslation   | 控件整体向Y轴的偏移量              | Number  |  | 0      |
| isUpperArm   | 是否为三连杆机械臂             | Boolean  |  true, false | false      |
| upperAngle   | 三连杆机械臂的情况下，最后一个连杆与X轴的夹角，正需要设置该参数，三连杆机械臂才能做逆运动学计算             | Number  |    | 45      |
| scale   | 缩放系数，将整体的控件进行缩放             | Number  |    | 1      |

#### RoboticPalmView

| Parameters       | Description                   | Type    | Available | Default    |
| ---------------- | ----------------------------- | ------- | --------- | ---------- |
| showBorder       | 显示画板的边界            | Boolean | -         | false      |
| canvasWidth      | 设置画板的宽度             | Number  | -         | 300        |
| canvasHeight     | 设置画板的高度             | Number  | -         | 350        |
| jonitColor       | 设置关节颜色               | String  |           | brown      |
| leverColor       | 设置连杆颜色               | String  |           | black      |
| endColor         | 设置末端关节颜色            | String  |           | aqua       |
| middleLeverWidth | 设置中间横杆的颜色        | Number  |           | 20         |
| leverWidth       | 设置连杆宽度               | Number  |           | 15         |
| jointRadius      | 设置关节的圆形半径              | Number  |           | 10         |
| withLabel        | 显示当前的文字颜色 | Boolean |           | false      |
| textColor        | 设置文字颜色    | String  |           | green      |
| fontStyle        | 设置文字的字体大小风格     | String  |           | 15px Arial |
| xTranslation   | 控件整体向X轴偏移量             | Number  |  | 0      |
| yTranslation   | 控件整体向Y轴的偏移量              | Number  |  | 0      |
| scale   | 缩放系数，将整体的控件进行缩放             | Number  |    | 1      |

#### Change Events

- RoboticArmView

| Event name       | Description                                                  | Callback parameters      |
| ---------------- | ------------------------------------------------------------ | ------------------------ |
| jointValueChange | 拖拽机械臂末端关节触发的事件 | {angle1: 0 , angle2:  0, angle3: 0} |

- RoboticPalmView

| Event name      | Description                                      | Callback parameters |
| --------------- | ------------------------------------------------ | ------------------- |
| palmValueChange | 拖拽机械爪触发的回调事件 | angle               |

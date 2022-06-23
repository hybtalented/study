Xacro 是一个基于 XML 的宏定义语言，通过 Xacro 可以以更简洁的、并且可读性更高的 XML 宏构建 XML。

Xacro 通常用于构建巨大的 XML 文档，例如机器人的定义文件中。在 urdf （Unified Robot description format） 中大量使用了 Xacro。下面我们将会学习 Xacro 中的基本语法。

# 属性和块属性
xacro 中属性和块属性是通过 `<xacro:property>` 标签定义的，其中属性用于将一个变量赋值到一个名称上，而块属性则用于指定一个名称来指代一个 XML 块。

属性的声明中，用于 `name` 属性指示属性的名称， `value` 属性指示属性的值，例如下面的代码定义了两个属性 `the_radius` 和 `the_length`， 其值分别为 `2.1` 和 `4.5`
```xml
<xacro:property name="the_radius" value="2.1" ></xacro:property>
<xacro:property name="the_length" value="4.5"></xacro:property>
```
在想要使用对应的属性值的时候，我们只要在对应的内容上使用 `${}`包括对应的变量即可
```xml
<geometry radius="${the_radius}" length="${the_length}" description="length is ${the_length}" />
```

块属性是一种特殊的属性，快属性的值为一个 xml 块。 在其声明时同样使用 `name` 指代块的名称，而其内容块作为属性的值。
```xml
<xacro:property name="front_left_origin">
    <origin xyz="0.3 0 0" rpy="0 0 0">
</xacro:property>
```
在上述代码中`front_left_origin` 用于指代`<origin xyz="0.3 0 0" rpy="0 0 0">`，
在我们想要使用块属性时，我们需要通过 `<xacro:insert_block>` 标签添加对应的块属性, 例如下述代码中，我们将 `front_left_origin`对应的块内容插入到了 `<pr_wheel_name name="front_left_wheel">` 标签中。
```xml
<pr_wheel_name name="front_left_wheel">
    <xacro:insert_block name="front_left_origin" />
</pr_wheel_name>
```
# 使用数学表达式
xml 标签的属性中可以通过 `${}` 输入一些基本的数学表达式，`xacro` 会对这些数学表达式进行计算。在数学表达式中可以使用属性、基本的数学运算以及一些常用的数学运算和常量。

在 ros 中（jade版本之后） 是通过 python 计算 `${}` 中的表达式，在表达式中可以使用 python 的 math 模块中定义的变量（如圆周率`pi`）和方法（如三角函数）, 下面的代码展示了一些常用的例子
```xml
<xacro:property name="radius" value="4.3" />
<xacro:property name="alpha" value="radians(60)" />
<xacro:property name="theta" value="45 / 18 * pi" />
<xacro:property name="phi" value="pi/3" />

<circle diameter="${2 * radius}" pos="${radius * cos(alpha)} ${radius * sin(alpha)}" />
<geometry  x="${radius * cos(theta) * cos(phi)}" y="${radius * cos(theta) * sin(phi)}" z="${radius * sin(theta)}"/>
```

# xacro 访问 yml 文件
前面说过 `xacro` 的 xml属性中的 `${}` 中内容是通过 python 解析的， 因此， `xacro` 也能够使用 python 中的一些类型，例如
```xml
<!-- 字典类型 -->
<xacro:property name="dict_var" value="${dict(a=1, b=2)}"/>
<xacro:property name="arr_var" value="${[1,2,3}" />
```
并且能够将 yaml 文件加载进来
```xml
<xacro:propery name="dict_yaml" value="${load_yaml(file.yaml)}" />
```
在 yaml 文件被加载后，将被视为一个字典。

# 条件表达式
在`xacro`中，可以使用 `<xacro:if>` 和 `<xacro:unless>` 两个标签控制xml块的启用和弃用，如下所示
```xml
<xacro:if value="<expression>">
  <!-- 条件成立时的 xml 块 -->
</xacro:if>
<xacro:unless value="<expression>">
  <!-- 条件不成立时的 xml 块 -->
</xacro:unless>
```
上述的 `<expression>` 为一个 python 表达式，并且表达式的值为 `1` 或者 `True` 时表示条件成立， `0` 或者 `False` 时条件不成立，其他值则报错。

下面列举一些常用的表达控制语句

```xml
<xacro:property name="var" value="useit" />
<xacro:property name="allowList" value="${[1,2,3]}" />
<!-- 常量 -->
<xacro:if value="1" />
<!-- python 表达式 -->
<xacro:if value="${var == 'useit'}" />
<xacro:if value="${var.startswith('use') and var.endswith('it')}" />
<xacro:if value="${1 in allowList}" />
```

# rospack 命令
在 `xacro` 中, 可以通过 `$()`  将xml 属性值设置为 rospack 命令的结果，例如
```xml
<foo value="$(find roscpp)" />
<!-- 需要通过命令行中添加 myvar:=xxx 设置下述命令的值 -->
<foo value="$(arg myvar)" />
```
`xacro` 基本上支持在 `$()` 中使用 roslaunch 的 [substitution args](http://wiki.ros.org/roslaunch/XML#substitution_args) 的所有大部分用法，除了 `eval`.

上述例子中 `$(arg myvar)` 表示从命令行中获取值，我们可以通过 `<xacro:arg>` 标签设置默认的值, 例如

```xml
<xacro:arg name="myvar" default="false" />
```
这时在命令行中未添加 `myvar:=xxx` 时， 参数 `myvar` 会被自动设置为 `false`。**args 参数被放在一个全局的字典中，因此可以在任意被包含的文件以及宏中获取。**

# 宏的使用
xacro 中一个重要的特性是可以声明一个宏。宏的定义是通过 `<xacro:macro>` 标签设置的，并且可以通过 `name` 属性设置宏的名称， `params` 指示宏的输入参数， 多个参数之间用空格分隔。例如下面的代码定义了一个 robot 宏
```xml
<xacro:macro name="robot" params="name location size" >
    <define:robot name="robot_${name}"/>
    <robot name="robot_${name}">
         <geometry x="${location[0]}" y="${location[1]}" z="${location[2]}" />
         <size x="${size[0]}" y="${size[1]}" z="${size[2]}"/>
    </robot>
</xacro:macro>
```
然后，我们可以通过 `<xacro:robot>` 标签去使用对应的宏

```xml
<xacro:robot name="test" location="${[0,0,1]}", size="${[1,1,1]}"/>
```
上述代码将被 `xacro` 展开为
```xml
<define:robot name="robot_test"/>
<robot name="robot_test">
    <geometry x="0" y="0" z="1" />
    <size x="1" y="1" z="1"/>
</robot>
```

在 xacro 的宏定义的参数声明中，可以在参数名称前添加星号(*) 表示插入的参数是一个块变量，添加一个星号表示对应的参数是一个xml块，添加两个星号表示对应的参数是多个xml块。例如下面的例子中定义了一个 `pr2_caster` 宏， 并且中宏中包含了一个普通参数 `suffix` 和 三个块参数 `origin，`、`content，` 和`anothercontent`
```xml
<xacro:macro name="pr2_caster" params="suffix *origin **content **anothercontent">
    <joint name="caster_${suffix}_joint">
       <axis xyz="0 0 1" />
    </joint>
    <link name="caster_${suffix}">
        <xacro:insert_block name="origin" />
        <xacro:insert_block name="content" />
        <xacro:insert_block name="anothercontent" />
    </link>
</xacro:macro>
```
如果，通过如下代码使用 `pr2_caster`宏
```xml
<xacro:pr2_caster suffix="front_left">
    <pose xyz="0 1 0" rpy="0 0 0" />
    <container>
        <color name="yellow"/>
        <mass>0.1</mass>
    </container>
    <another>
        <inertial>
          <origin xyz="0 0 0.5" rpy="0 0 0"/>
          <mass value="1"/>
         <inertia ixx="100"  ixy="0"  ixz="0" iyy="100" iyz="0" izz="100" />
        </inertial>
    </another>
</xacro:pr2_caster>
```
经过 `xacro` 展开以后，将会得到如下的 xml 

```xml
<joint name="caster_front_left_joint">
    <axis xyz="0 0 1" />
</joint>
<link name="caster_front_left">
    <pose xyz="0 1 0" rpy="0 0 0" />
    <color name="yellow"/>
    <mass>0.1</mass>
    <inertial>
        <origin xyz="0 0 0.5" rpy="0 0 0"/>
        <mass value="1"/>
        <inertia ixx="100"  ixy="0"  ixz="0" iyy="100" iyz="0" izz="100" />
    </inertial>
</link>
```
在宏使用时，如果块参数（即在参数的名称添加了星号）如果未在对应的宏标签（如上例中的 `<xacro:pr2_caster>`）的属性上声明，将会依次从宏标签的内容块中获取一个子标签，赋值给对应的参数，特别的是，如果参数名称添加了两个星号，其值被设置为对应子标签的内容块。

在 xacro 的宏声明时，可以给参数设置默认的值
```xml
<xacro:macro name="foo" params="x:=${x} y:=${2*y} z:=${0}" />
```
特别的是，如果参数使用了外部作用域的同名变量作为默认值，可以将表达式写为 `^`, 例如上述例子可以简写为

```xml
<xacro:macro name="foo" params="x:=^ y:=${2*y} z:=${0}" />
```
如果还需要在外部作用域未定义对应的属性时，使用另外一个表达式作为默认值，可以将默认值设置为`^|表达式`, 例如
```xml
<xacro:macro name="foo" params="x:=^|${0} y:=${2*y} z:=${0}" />
```

在宏中定义的属性和子宏是局部的，只能在对应的该宏中使用，对于宏的外部是不可见的，如果想要在宏中，想外部注入变量，可以在对应的属性的声明是添加`scope` xml 属性。 如果 `scope` 的值被设置为 `parent` 属性将被注入到宏的父作用域；如果 `scope` 的值被设置为 `global`， 属性将被注入到全局作用域。

# 包含其他的 xacro 文件

在 `xacro` 中， 可以通过 `<xacro:include>`将其他的 `xacro` 文件包含进来，

```xml
<xacro:include filename="other.xacro" />
<xacro:include filename="$(find pkg)/pkg.xacro" />
<xacro:include filename="$(cwd)/other.xacro" />
```

如果包含的文件是相对路径的话，则路径相对于当前的 `xacro` 文件。除此之外， 为了避免包含的文件中的属性和宏的命名冲突，可以在 `<xacro:include>` 标签上添加 `ns` xml 属性，从而可在通过 `ns` 命名空间去访问对应的属性和宏。

```xml
<xacro:include filename="other.xacro" ns="other" />

<!-- 访问属性 -->
<foo value="${other.property}" />
<!-- 访问宏 -->
<xacro:other.macro />
```
**需要注意的是：如果在一个宏中通过<xacro:include>包含其他的 `xacro` 文件， 文件将会在宏调用的时候展开，而不是在宏定义的时候展开。**

# 动态属性和动态标签
如果你想要通过动态的变量生成一个 xml 标签，或者一个 xml 标签的属性名，可以通过 `<xacro:element>` 和 `<xacro:attribute>` 即可实现
```xml
<xacro:element xacro:name="tag_name" [other attribute] />
<xacro:attirbute name="${attr_name}" value="表达式" />
```
例如 
```xml
<xacro:property name="test_tag" value="test" />
<xacro:element xacro:name="${test_tag}" test="1" />
```
将会被 `xacro` 展开为
```xml
<test test="1" />
```
而
```xml
<xacro:property name="test_attr" value="test" />
<tag> 
    <xacro:attribute name="${test_attr}" value="2">
</tag>
```
将会被 `xacro` 展开为
```xml
<tag test="2" />
```

# CMakeLists 处理 xacro 文件
下面为将 xacro 文件添加到 CMake 的构建目标中的示例代码，

```cmake
find_package(xacro REQUIRED)
# 如果是通过 catkin_make 进行构建的话，也可以使用一下的代码将 xacro 包含进来
# find_package(catkin REQUIRED COMPONENTS ... xacro)

# 添加需要被 xacro 编译的xacro文件
file(GLOB xacro_files ${CMAKE_CURRENT_SOURCE_DIR}/world/*.world.xacro)

foreach(file  ${xacro_files}) 
    # 剔除 file 文件中的 .xacro 后缀
    string(REGEX MATCH "(.*)[.]xacro$" unused ${file})
    # 获取剔除后缀后的文件
    set(out_file ${CMAKE_MATCH_1})

    # 使用 xacro 的 cmake 宏创建目标
    xacro_add_xacro_file(${file} ${out_file})
    
    # xacro_add_xacro_file 会自动创建一个生成目标，这里将对应的目标添加到一个数组里
    list(APPEND world_file ${out_file})
endforeach(file)

# 最后我们将所有的 xacro 构建目标合并为一个目标, 通过该构建目标可以实现所有的 xacro 文件的构建
add_custom_target(media_files ALL DEPENDS ${world_file})
```
事实上，上述的代码有更简单的实现方式
```cmake
find_package(xacro REQUIRED)
file(GLOB xacro_files ${CMAKE_CURRENT_SOURCE_DIR}/world/*.world.xacro)

xacro_add_files(${xacro_files} TARGET media_files)
```
上述代码会将 `.xacro` 后缀的文件通过 `xacro` 编译后生成对应的目标 xml 文件，并且 xml 文件的文件名会去除掉 `.xacro` 后缀, 因此, 我们只要根据我们想要生成的文件名，将对应的 xacro 文件的文件名设置为 `目标文件名.xacro`即可。

# xacro 的生成过程解析

在旧版 xacro 中， 首先会将所有的 `<xacro:include>` 标签加载并展开， 然后将会处理所有的属性和宏定义， 最后才会把展开宏调用以及计算相关表达式。这导致了一个结果，就是无法通过 xacro 条件控制标签控制属性和宏的定义，以及其他 xacro 文件的包含。

而在 jade 版本的 ros 中， xacro 可以通过 --inorder 参数，提供了一种新的文档处理方式。这种处理方式将会安装 XML 文档流 的顺序处理 `xacro` 标签，并产生了以下的特性
1. 宏调用和表达式的计算计算的时候将会使用最新的属性和宏而不是最后的属性和宏；
2. 控制语句和 `xacro` 宏中的 `<xacro:include>` 标签，只会在宏被调用或者控制语句成立时才会被展开，并且可以将`<xacro:include>`  的文件名设置为一个变量
3. 在全局文档中有一个全局作用域，而且没一个宏中也有自己的局部作用域，局部作用域中的变量设置不会影响全局作用域的变量。

**在 jade 之后的版本中，不再需要添加 --inorder， 默认将使用新的处理方式。**
# C++ 代码规范

### 基础规则

遵循 [Google C++ 风格指南](https://google.github.io/styleguide/cppguide.html)，并遵循以下补充规则：

#### 1. This 指针

所有类属性和成员函数必须通过 `this->` 指针访问：

```cpp
class MyClass
{
public:
  void SetValue(int _value)
  {
    this->value = _value;  // Good
  }

  int GetValue() const
  {
    return this->value;    // Good
  }

private:
  int value;
};
```

#### 2. 函数参数下划线前缀

所有函数参数必须以下划线开头：

```cpp
void MyClass::MyFunction(int _value, const std::string &_name)
{
  this->value = _value;
  this->name = _name;
}
```

#### 3. 大括号独占一行

所有大括号 `{}` 必须独占一行，包括控制流语句：

```cpp
// Good
if (this->enabled)
{
  this->DoSomething();
}

// Bad - 不允许
if (this->enabled) {
  this->DoSomething();
}
```

#### 4. 多行代码块

流控制语句（`if`, `for`, `while` 等）的代码块必须用大括号包裹：

```cpp
// Good
for (int i = 0; i < 10; ++i)
{
  this->Process(i);
}

// Bad - 不允许
for (int i = 0; i < 10; ++i)
  this->Process(i);
```

#### 5. 前缀递增运算符

使用前缀形式 `++i` 而不是 `i++`（在 for 循环中特别重要）：

```cpp
// Good
for (int i = 0; i < 10; ++i)
{
  // ...
}

// Bad
for (int i = 0; i < 10; i++)
{
  // ...
}
```

#### 6. PIMPL 模式（私有数据指针）

所有新类必须使用 PIMPL 模式隐藏实现细节：

```cpp
// In header file
class MyClass
{
public:
  MyClass();
  ~MyClass();

private:
  class MyClassPrivate;
  std::unique_ptr<MyClassPrivate> this->dataPtr;
};
```

#### 7. Const 成员函数

不修改成员变量的成员函数必须标记为 `const`：

```cpp
class MyClass
{
public:
  // Good - 只读，标记为 const
  int GetValue() const
  {
    return this->value;
  }

  // Good - 修改成员，不是 const
  void SetValue(int _value)
  {
    this->value = _value;
  }
};
```

#### 8. Const 参数

非 POD 类型的参数必须标记为 `const`（除了输出参数）：

```cpp
// Good
void MyClass::Process(const std::string &_name, int _count)
{
}

// Bad - 字符串应该是 const
void MyClass::Process(std::string &_name, int _count)
{
}
```

#### 9. 指针和引用位置

`*` 和 `&` 紧邻变量名，而不是类型：

```cpp
// Good
int &variable = reference;
int *pointer = nullptr;

// Bad
int& variable = reference;
int* pointer = nullptr;
```

#### 10. 命名规范

**类名和成员函数**：驼峰命名，首字母大写
```cpp
class MyClass
{
public:
  void MyFunction();
  void ProcessData();
};
```

**自由函数**：驼峰命名，首字母小写
```cpp
namespace my_namespace
{
  void myFunction();
  void processData();
}
```

**变量**：驼峰命名，首字母小写
```cpp
int myVariable;
std::string myName;
```

#### 11. 禁止行内注释

`//` 注释不能与代码同行：

```cpp
// Good
// Convert miles per hour to meters per second
speed *= 0.44704;

// Bad - 不允许
speed *= 0.44704;  // miles per hour to meters per second
```

#### 12. 访问器命名

成员访问器必须遵循命名规范，不使用 `Get` 前缀：

```cpp
class ServerConfig
{
public:
  // Good - 使用名词形式
  ServerConfig Config() const;
  void SetConfig(const ServerConfig &_config);

  // Bad - 不允许 Get 前缀
  ServerConfig GetConfig() const;
};
```

**特殊情况**：
- 名称与类冲突时，使用 `Noun-By` 格式：
  ```cpp
  ModelByName(const std::string &_name);
  ModelById(const int _id);
  ```

- 模板函数可使用 `Get`：
  ```cpp
  template<typename T>
  T Get();
  ```

#### 13. 修改器命名

写入访问器必须以 `Set` 开头：

```cpp
// Good
void SetServerConfig(const ServerConfig &_config);

// Bad
void ServerConfig(const ServerConfig &_config);
```

---


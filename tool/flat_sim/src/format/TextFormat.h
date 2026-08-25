// flat_sim —— 自研格式的极简文本解析（YAML 风格子集）
//
// 支持的语法（故意保持最小，不引入外部 YAML 库）：
//   - 缩进块：`块名:` 在前，子字段缩进 2 空格（或多空格）写在下面
//   - 字段：`key: value`，value 为标量或同行列表 `[a, b, c]`
//   - 备注：整行 `# ...` 与行尾 ` # ...`，双引号字符串里的 # 保留；
//           解析时备注被直接丢弃，绝不进入运行时模型
//   - 键名大小写不敏感（统一按小写比较）
// 缩进请用空格，不要用 Tab。
#pragma once

#include <stdexcept>
#include <string>
#include <vector>

namespace flat_sim {
namespace format {

// 带文件与行号的解析错误（失败可见：报错精确到行）
struct FormatError : std::runtime_error {
  std::string file;
  int line;
  FormatError(const std::string& file, int line, const std::string& msg);
};

// 解析树节点
struct Node {
  std::string key;                  // 小写键名；文档根节点为 "#document"
  std::string value;                // 标量值（已去引号）；块节点为空
  bool hasValue = false;
  bool isList = false;              // value 形如 [a, b, c]
  std::vector<std::string> list;    // isList 时的各元素（已去引号）
  int line = 0;                     // 所在行号（1 起）
  std::vector<Node> children;

  const Node* find(const std::string& key) const;          // 第一个匹配子节点
  std::vector<const Node*> findAll(const std::string& key) const;
};

// 把文本解析成节点树；语法错误抛 FormatError
Node parseText(const std::string& text, const std::string& filename);

// 取值工具（错误均带 file:line）
std::string scalarValue(const Node& n, const std::string& file);
double toDouble(const Node& n, const std::string& file);
int toInt(const Node& n, const std::string& file);
// expect = 0 表示不限定元素个数
std::vector<double> toDoubleList(const Node& n, size_t expect, const std::string& file);

}  // namespace format
}  // namespace flat_sim

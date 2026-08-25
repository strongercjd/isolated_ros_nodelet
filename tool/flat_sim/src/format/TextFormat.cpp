// flat_sim —— 极简文本解析实现
#include "format/TextFormat.h"

#include <cctype>
#include <sstream>

namespace flat_sim {
namespace format {

FormatError::FormatError(const std::string& file, int line, const std::string& msg)
    : std::runtime_error(file + ":" + std::to_string(line) + ": " + msg),
      file(file),
      line(line) {}

namespace {

std::string trim(const std::string& s) {
  const size_t b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos) return "";
  const size_t e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}

std::string lower(std::string s) {
  for (char& c : s) c = (char)std::tolower((unsigned char)c);
  return s;
}

std::string unquote(const std::string& s) {
  if (s.size() >= 2 && s.front() == '"' && s.back() == '"') return s.substr(1, s.size() - 2);
  return s;
}

// 去掉 # 备注：整行与行尾均支持；双引号内的 # 保留（备注规则见需求 4.2）
std::string stripComment(const std::string& raw) {
  bool inQuote = false;
  for (size_t i = 0; i < raw.size(); ++i) {
    if (raw[i] == '"') inQuote = !inQuote;
    else if (raw[i] == '#' && !inQuote) return raw.substr(0, i);
  }
  return raw;
}

double parseDouble(const std::string& s) {
  size_t pos = 0;
  double v = std::stod(s, &pos);  // 抛 invalid_argument / out_of_range
  if (pos != s.size()) throw std::invalid_argument("trailing");
  return v;
}

}  // namespace

const Node* Node::find(const std::string& k) const {
  for (const Node& c : children)
    if (c.key == k) return &c;
  return nullptr;
}

std::vector<const Node*> Node::findAll(const std::string& k) const {
  std::vector<const Node*> out;
  for (const Node& c : children)
    if (c.key == k) out.push_back(&c);
  return out;
}

Node parseText(const std::string& text, const std::string& filename) {
  Node root;
  root.key = "#document";
  struct Frame {
    int indent;
    Node* node;
  };
  std::vector<Frame> stack{{-1, &root}};

  std::istringstream in(text);
  std::string raw;
  int lineno = 0;
  while (std::getline(in, raw)) {
    ++lineno;

    // 1) 丢备注、算缩进（纯备注 / 空行直接跳过，不进树）
    const std::string noComment = stripComment(raw);
    int indent = 0;
    size_t i = 0;
    for (; i < noComment.size() && (noComment[i] == ' ' || noComment[i] == '\t'); ++i) {
      if (noComment[i] == '\t') throw FormatError(filename, lineno, "缩进请用空格（不要用 Tab）");
      ++indent;
    }
    const std::string content = trim(noComment);
    if (content.empty()) continue;

    // 2) key: value
    const size_t colon = content.find(':');
    if (colon == std::string::npos || colon == 0)
      throw FormatError(filename, lineno,
                        "格式应为 `字段: 值`（缺少冒号或字段名为空）: \"" + content + "\"");
    Node node;
    node.line = lineno;
    node.key = lower(trim(content.substr(0, colon)));
    const std::string value = trim(content.substr(colon + 1));
    if (!value.empty()) {
      node.hasValue = true;
      if (value.front() == '[') {
        if (value.back() != ']')
          throw FormatError(filename, lineno,
                            "列表 " + value + " 缺少右括号 ]（列表必须写在同一行）");
        node.isList = true;
        std::string item;
        std::stringstream ss(value.substr(1, value.size() - 2));
        while (std::getline(ss, item, ',')) node.list.push_back(unquote(trim(item)));
      } else {
        node.value = unquote(value);
      }
    }

    // 3) 挂树：弹出到第一个缩进更小的层
    while (stack.back().indent >= indent) stack.pop_back();
    stack.back().node->children.push_back(std::move(node));
    // 无论有无行内值都入栈：块节点可继续嵌套；
    // 带值节点也允许子字段（robot_file: "xx.frobot" 下写 pose 覆盖）。
    Node* placed = &stack.back().node->children.back();
    stack.push_back(Frame{indent, placed});
  }
  return root;
}

std::string scalarValue(const Node& n, const std::string& file) {
  if (!n.hasValue || n.isList)
    throw FormatError(file, n.line, "`" + n.key + "` 应为标量（不是列表）");
  return n.value;
}

double toDouble(const Node& n, const std::string& file) {
  try {
    return parseDouble(n.value);
  } catch (const std::exception&) {
    throw FormatError(file, n.line, "`" + n.key + "` 不是合法数值: \"" + n.value + "\"");
  }
}

int toInt(const Node& n, const std::string& file) {
  double v = 0;
  try {
    v = parseDouble(n.value);
  } catch (const std::exception&) {
    throw FormatError(file, n.line, "`" + n.key + "` 不是合法整数: \"" + n.value + "\"");
  }
  const int iv = (int)v;
  if ((double)iv != v)
    throw FormatError(file, n.line, "`" + n.key + "` 应为整数，实际: " + n.value);
  return iv;
}

std::vector<double> toDoubleList(const Node& n, size_t expect, const std::string& file) {
  if (!n.isList)
    throw FormatError(file, n.line, "`" + n.key + "` 应写成列表 [a, b, ...]");
  std::vector<double> out;
  for (size_t i = 0; i < n.list.size(); ++i) {
    try {
      out.push_back(parseDouble(n.list[i]));
    } catch (const std::exception&) {
      throw FormatError(file, n.line,
                        "`" + n.key + "` 第 " + std::to_string(i + 1) + " 个元素不是数值: \"" +
                            n.list[i] + "\"");
    }
  }
  if (expect != 0 && out.size() != expect)
    throw FormatError(file, n.line, "`" + n.key + "` 应有 " + std::to_string(expect) +
                                        " 个元素，实际 " + std::to_string(out.size()));
  return out;
}

}  // namespace format
}  // namespace flat_sim

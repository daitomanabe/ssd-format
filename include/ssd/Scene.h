#pragma once
// SSD v0.1 reference reader. C++17, standard library only.
#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <fstream>
#include <functional>
#include <locale>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace ssd {
struct Vec3 {
    double x = 0, y = 0, z = 0;
    Vec3 operator+(Vec3 b) const {
        return {x + b.x, y + b.y, z + b.z};
    }
    Vec3 operator-(Vec3 b) const {
        return {x - b.x, y - b.y, z - b.z};
    }
    Vec3 operator*(double s) const {
        return {x * s, y * s, z * s};
    }
};
struct Mat4 {
    std::array<double, 16> m{1, 0, 0, 0, 0, 1, 0, 0,
                             0, 0, 1, 0, 0, 0, 0, 1}; // row major, column vectors
    Vec3 point(Vec3 p) const {
        return {m[0] * p.x + m[1] * p.y + m[2] * p.z + m[3],
                m[4] * p.x + m[5] * p.y + m[6] * p.z + m[7],
                m[8] * p.x + m[9] * p.y + m[10] * p.z + m[11]};
    }
    Mat4 operator*(const Mat4 &b) const {
        Mat4 r;
        r.m.fill(0);
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                for (int k = 0; k < 4; k++)
                    r.m[i * 4 + j] += m[i * 4 + k] * b.m[k * 4 + j];
        return r;
    }
    static Mat4 translation(Vec3 p) {
        Mat4 r;
        r.m[3] = p.x;
        r.m[7] = p.y;
        r.m[11] = p.z;
        return r;
    }
    static Mat4 rotation(int axis, double deg) {
        Mat4 r;
        int a = (axis + 1) % 3, b = (axis + 2) % 3;
        double t = deg * 3.14159265358979323846 / 180, c = std::cos(t), s = std::sin(t);
        r.m[a * 4 + a] = c;
        r.m[b * 4 + b] = c;
        r.m[a * 4 + b] = -s;
        r.m[b * 4 + a] = s;
        return r;
    }
};
inline Mat4 pose(Vec3 p, Vec3 ypr) {
    return Mat4::translation(p) * Mat4::rotation(1, ypr.z) * Mat4::rotation(0, ypr.y) *
           Mat4::rotation(2, ypr.x);
}
inline Vec3 toOpenFrameworks(Vec3 p) {
    return {p.x, p.z, -p.y};
}
struct Object {
    std::string id, type, name, parent;
    Vec3 position, angles;
    bool enabled = true;
};
struct Row {
    std::vector<std::string> cells;
    size_t line;
    const std::string &operator[](size_t i) const {
        return cells.at(i);
    }
};
struct Scene {
    std::string original;
    std::map<std::string, std::string> properties;
    std::map<std::string, std::vector<Row>> sections;
    std::map<std::string, Object> objects;
    std::vector<std::string> warnings;
    std::map<std::string, Mat4> worldTransforms;
    std::map<std::string, bool> worldEnabled;
    std::string property(const std::string &key, const std::string &fallback = "") const {
        auto i = properties.find(key);
        return i == properties.end() ? fallback : i->second;
    }
    const std::vector<Row> &rows(const std::string &name) const {
        static const std::vector<Row> empty;
        auto i = sections.find(name);
        return i == sections.end() ? empty : i->second;
    }
    const Row *row(const std::string &section, const std::string &id) const {
        for (auto &r : rows(section))
            if (!r.cells.empty() && r[0] == id)
                return &r;
        return nullptr;
    }
    Mat4 world(const std::string &id) const {
        auto i = worldTransforms.find(id);
        if (i == worldTransforms.end())
            throw std::runtime_error("Unknown object ID for world transform: " + id);
        return i->second;
    }
    bool active(const std::string &id) const {
        auto i = worldEnabled.find(id);
        if (i == worldEnabled.end())
            throw std::runtime_error("Unknown object ID for active state: " + id);
        return i->second;
    }
    std::string serialize() const {
        return original;
    } // preserve comments, unknown extensions, BOM and line endings byte-for-byte
    Vec3 uvToWorld(const std::string &id, double u, double v) const;
};
inline std::string trim(std::string s) {
    auto a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos)
        return "";
    return s.substr(a, s.find_last_not_of(" \t\r\n") - a + 1);
}
inline std::vector<std::string> split(const std::string &s) {
    std::vector<std::string> r;
    size_t a = 0, b;
    while ((b = s.find('\t', a)) != std::string::npos) {
        r.push_back(s.substr(a, b - a));
        a = b + 1;
    }
    r.push_back(s.substr(a));
    return r;
}
[[noreturn]] inline void fail(const Row &r, const std::string &s) {
    throw std::runtime_error("line " + std::to_string(r.line) + ": " + s);
}
inline bool decimalSyntax(const std::string &s) {
    size_t i = !s.empty() && s[0] == '-' ? 1 : 0;
    auto digits = [&] {
        size_t start = i;
        while (i < s.size() && s[i] >= '0' && s[i] <= '9')
            ++i;
        return i > start;
    };
    bool mantissa = digits();
    if (i < s.size() && s[i] == '.') {
        ++i;
        mantissa = digits() || mantissa;
    }
    if (!mantissa)
        return false;
    if (i < s.size() && (s[i] == 'e' || s[i] == 'E')) {
        ++i;
        if (i < s.size() && (s[i] == '+' || s[i] == '-'))
            ++i;
        if (!digits())
            return false;
    }
    return i == s.size();
}
inline double number(const Row &r, size_t i) {
    if (i >= r.cells.size() || r[i].empty())
        fail(r, "missing numeric field");
    double v = 0;
    const auto &cell = r[i];
    if (!decimalSyntax(cell))
        fail(r, "invalid finite decimal number: " + cell);
#if defined(SSD_TEST_CLASSIC_NUMBER_PARSER) || \
    (defined(__APPLE__) && defined(__ENVIRONMENT_MAC_OS_X_VERSION_MIN_REQUIRED__) && \
     __ENVIRONMENT_MAC_OS_X_VERSION_MIN_REQUIRED__ < 260000)
    // libc++ floating from_chars is available only on macOS 26+. Preserve the viewer's
    // deployment target with the same decimal grammar and an explicit classic locale.
    std::istringstream input(cell);
    input.imbue(std::locale::classic());
    input >> std::noskipws >> v;
    if (!input || input.peek() != std::char_traits<char>::eof() || !std::isfinite(v))
        fail(r, "invalid finite decimal number: " + cell);
    auto exponent = cell.find_first_of("eE");
    auto mantissaEnd = exponent == std::string::npos ? cell.end() : cell.begin() + exponent;
    if (v == 0 && std::any_of(cell.begin(), mantissaEnd, [](char c) { return c >= '1' && c <= '9'; }))
        fail(r, "invalid finite decimal number: " + cell); // nonzero value underflowed to zero
#else
    auto result = std::from_chars(cell.data(), cell.data() + cell.size(), v,
                                  std::chars_format::general);
    if (result.ec != std::errc{} || result.ptr != cell.data() + cell.size() || !std::isfinite(v))
        fail(r, "invalid finite decimal number: " + cell);
#endif
    return v;
}
inline void positive(const Row &r, size_t i) {
    if (number(r, i) <= 0)
        fail(r, "dimension must be positive");
}
inline void integer(const Row &r, size_t i, bool allowZero = true) {
    double v = number(r, i);
    if (v < (allowZero ? 0 : 1) || v > 2147483647.0 || std::floor(v) != v)
        fail(r, "expected integer in [" + std::string(allowZero ? "0" : "1") + ",2147483647]");
}
inline bool flag(const Row &r, size_t i) {
    if (r[i] != "0" && r[i] != "1")
        fail(r, "flag must be 0 or 1");
    return r[i] == "1";
}
inline void width(const Row &r, size_t n) {
    if (r.cells.size() < n)
        fail(r, "expected at least " + std::to_string(n) + " tab-separated fields");
}
inline void validate(Scene &s) {
    for (auto key : {"Version", "Unit", "CoordinateSystem", "AngleUnit"})
        if (!s.properties.count(key))
            throw std::runtime_error(std::string("SCENE missing ") + key);
    if (s.property("Version") != "0.1" || s.property("Unit") != "meter" ||
        s.property("CoordinateSystem") != "SSD_RH_ZUP" || s.property("AngleUnit") != "degree")
        throw std::runtime_error("Unsupported SCENE version/unit/coordinate/angle profile");
    std::map<std::string, const Row *> objectRows;
    for (auto &r : s.rows("OBJECT")) {
        width(r, 11);
        if (r[0].empty() || r[0] == "none" || r[1].empty())
            fail(r, "invalid object ID/type");
        Object o{r[0],
                 r[1],
                 r[2],
                 r[3],
                 {number(r, 4), number(r, 5), number(r, 6)},
                 {number(r, 7), number(r, 8), number(r, 9)},
                 flag(r, 10)};
        if (!s.objects.emplace(o.id, o).second)
            fail(r, "duplicate object ID: " + o.id);
        objectRows[o.id] = &r;
    }
    std::map<std::string, int> state;
    std::map<std::string, size_t> parentDepth;
    std::function<void(const std::string &, size_t)> visit = [&](const std::string &id,
                                                                 size_t depth) {
        if (depth > 512)
            fail(*objectRows.at(id), "object " + id + ": parent hierarchy exceeds reader depth limit 512");
        if (state[id] == 1)
            fail(*objectRows.at(id), "object " + id + ": parent cycle");
        if (state[id] == 2)
            return;
        state[id] = 1;
        const auto &o = s.objects.at(id);
        Mat4 m = pose(o.position, o.angles);
        bool enabled = o.enabled;
        if (o.parent != "none") {
            if (!s.objects.count(o.parent))
                fail(*objectRows.at(id), "object " + id + ": missing parent " + o.parent);
            visit(o.parent, depth + 1);
            parentDepth[id] = parentDepth.at(o.parent) + 1;
            if (parentDepth[id] > 512)
                fail(*objectRows.at(id), "object " + id + ": parent hierarchy exceeds reader depth limit 512");
            m = s.worldTransforms.at(o.parent) * m;
            enabled = enabled && s.worldEnabled.at(o.parent);
        } else
            parentDepth[id] = 0;
        s.worldTransforms[id] = m;
        s.worldEnabled[id] = enabled;
        state[id] = 2;
    };
    for (auto &pair : s.objects)
        visit(pair.first, 0);
    const std::map<std::string, size_t> counts = {{"SCREEN", 3},    {"SURFACE", 3},
                                                  {"LED", 5},       {"DISPLAY", 4},
                                                  {"PROJECTOR", 5}, {"PIXELMAP", 8},
                                                  {"SPEAKER", 5},   {"MICROPHONE", 3},
                                                  {"SENSOR", 2},    {"CAMERA", 5},
                                                  {"TRACKER", 3},   {"LIGHT", 4},
                                                  {"ROBOT", 3},     {"BOX", 4},
                                                  {"EVIDENCE", 6},  {"FOV", 5},
                                                  {"DEVICE", 3},    {"VISUAL_REQUIREMENT", 4},
                                                  {"INVENTORY", 6}, {"SHARED_INVENTORY", 4}};
    const std::set<std::string> virtualSections = {"DISPLAY", "PIXELMAP", "INVENTORY",
                                                   "SHARED_INVENTORY"};
    auto objectRef = [&](const Row &r, size_t i) {
        if (!s.objects.count(r[i]))
            fail(r, "missing object reference: " + r[i]);
    };
    auto visualRef = [&](const Row &r, size_t i) {
        objectRef(r, i);
        auto t = s.objects.at(r[i]).type;
        if (t != "screen" && t != "surface" && t != "led")
            fail(r, "target is not screen/surface/led");
    };
    for (auto &section : counts) {
        std::set<std::string> ids;
        for (auto &r : s.rows(section.first)) {
            width(r, section.second);
            if (r[0].empty() || !ids.insert(r[0]).second)
                fail(r, "empty or duplicate ID in " + section.first);
            if (!virtualSections.count(section.first))
                objectRef(r, 0);
            std::string n = section.first;
            const std::map<std::string, std::string> types = {
                {"SCREEN", "screen"},       {"SURFACE", "surface"}, {"LED", "led"},
                {"PROJECTOR", "projector"}, {"SPEAKER", "speaker"}, {"MICROPHONE", "microphone"},
                {"SENSOR", "sensor"},       {"CAMERA", "camera"},   {"TRACKER", "tracker"},
                {"LIGHT", "light"},         {"ROBOT", "robot"}};
            if (types.count(n) && s.objects.at(r[0]).type != types.at(n))
                fail(r, "object type disagrees with " + n);
            if (n == "SCREEN" || n == "SURFACE" || n == "LED") {
                positive(r, 1);
                positive(r, 2);
            }
            if (n == "LED") {
                integer(r, 3, false);
                integer(r, 4, false);
                if (r.cells.size() > 5 && !r[5].empty())
                    positive(r, 5);
            }
            if (n == "DISPLAY") {
                integer(r, 2, false);
                integer(r, 3, false);
            }
            if (n == "PROJECTOR") {
                if (!s.row("DISPLAY", r[1]))
                    fail(r, "missing display: " + r[1]);
                visualRef(r, 2);
                integer(r, 3, false);
                integer(r, 4, false);
            }
            if (n == "PIXELMAP") {
                auto d = s.row("DISPLAY", r[1]);
                if (!d)
                    fail(r, "missing display: " + r[1]);
                width(*d, 4);
                visualRef(r, 2);
                for (size_t i = 3; i <= 6; i++)
                    integer(r, i, i < 5);
                double rot = number(r, 7);
                if (rot != 0 && rot != 90 && rot != 180 && rot != 270)
                    fail(r, "unsupported pixel rotation");
                if (number(r, 3) + number(r, 5) > number(*d, 2) ||
                    number(r, 4) + number(r, 6) > number(*d, 3))
                    fail(r, "pixel rectangle outside display");
            }
            if (n == "SPEAKER") {
                integer(r, 1, false);
                number(r, 2);
                if (number(r, 3) < 0)
                    fail(r, "negative delay");
                flag(r, 4);
            }
            if (n == "CAMERA") {
                for (size_t i = 1; i <= 2; i++) {
                    double v = number(r, i);
                    if (v <= 0 || v >= 180)
                        fail(r, "invalid camera FOV");
                }
                integer(r, 3, false);
                integer(r, 4, false);
            }
            if (n == "MICROPHONE")
                integer(r, 1, false);
            if (n == "LIGHT") {
                integer(r, 1);
                integer(r, 2, false);
                if (number(r, 2) > 512)
                    fail(r, "DMX address exceeds 512");
            }
            if (n == "BOX") {
                positive(r, 1);
                positive(r, 2);
                positive(r, 3);
            }
            if (n == "FOV") {
                for (size_t i = 1; i <= 2; i++) {
                    double v = number(r, i);
                    if (v <= 0 || v >= 180)
                        fail(r, "invalid FOV");
                }
                positive(r, 3);
            }
        }
    }
    const std::set<std::string> custom = {"INVENTORY", "SHARED_INVENTORY", "UNRESOLVED",
                                          "REVIEW_VOLUME"};
    for (auto &sec : s.sections)
        if (sec.first != "SCENE" && sec.first != "OBJECT" && !counts.count(sec.first) &&
            !custom.count(sec.first))
            s.warnings.push_back("Preserved unknown section [" + sec.first + "]");
}
inline bool validUtf8(const std::string &s) {
    for (size_t i = 0; i < s.size();) {
        unsigned char c = s[i++];
        if (c < 0x80)
            continue;
        int n = 0;
        unsigned cp = 0;
        if (c >= 0xC2 && c <= 0xDF) {
            n = 1;
            cp = c & 31;
        } else if (c >= 0xE0 && c <= 0xEF) {
            n = 2;
            cp = c & 15;
        } else if (c >= 0xF0 && c <= 0xF4) {
            n = 3;
            cp = c & 7;
        } else
            return false;
        if (i + n > s.size())
            return false;
        for (int k = 0; k < n; k++) {
            unsigned char v = s[i++];
            if ((v & 0xC0) != 0x80)
                return false;
            cp = (cp << 6) | (v & 63);
        }
        if ((n == 1 && cp < 0x80) || (n == 2 && cp < 0x800) || (n == 3 && cp < 0x10000) ||
            cp > 0x10FFFF || (cp >= 0xD800 && cp <= 0xDFFF))
            return false;
    }
    return true;
}
inline Scene parse(const std::string &data) {
    if (!validUtf8(data))
        throw std::runtime_error("Scene must be UTF-8");
    Scene s;
    s.original = data;
    std::istringstream input(data);
    std::string line, current;
    size_t n = 0;
    bool seen = false;
    while (std::getline(input, line)) {
        ++n;
        if (n == 1 && line.compare(0, 3, "\xEF\xBB\xBF") == 0)
            line.erase(0, 3);
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        std::string t = trim(line);
        if (t.empty() || t[0] == '#')
            continue;
        if (t.front() == '[') {
            if (t.back() != ']' || t.size() < 3 || t.find('[', 1) != std::string::npos ||
                t.find(']') != t.size() - 1)
                fail(Row{{t}, n}, "malformed section header: " + t);
            current = t.substr(1, t.size() - 2);
            if (!seen && current != "SCENE")
                throw std::runtime_error("First section must be SCENE");
            seen = true;
            s.sections[current];
            continue;
        }
        Row r{split(line), n};
        if (current.empty())
            fail(r, "data outside section");
        if (current == "SCENE") {
            width(r, 2);
            if (!s.properties.emplace(r[0], r[1]).second)
                fail(r, "duplicate SCENE key");
        } else
            s.sections[current].push_back(r);
    }
    validate(s);
    return s;
}
inline Scene load(const std::string &path) {
    std::ifstream f(path, std::ios::binary);
    if (!f)
        throw std::runtime_error("Cannot open scene: " + path);
    std::ostringstream b;
    b << f.rdbuf();
    return parse(b.str());
}
inline Vec3 Scene::uvToWorld(const std::string &id, double u, double v) const {
    if (!std::isfinite(u) || !std::isfinite(v) || u < 0 || u > 1 || v < 0 || v > 1)
        throw std::runtime_error("UV must be finite and in [0,1]");
    const Row *r = nullptr;
    for (auto name : {"SCREEN", "SURFACE", "LED"})
        if ((r = row(name, id)))
            break;
    if (!r)
        throw std::runtime_error("Object has no rectangular visual dimensions: " + id);
    return world(id).point({(u - .5) * number(*r, 1), (v - .5) * number(*r, 2), 0});
}
} // namespace ssd

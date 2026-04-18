// Analytical global placer using quadratic wirelength and recursive bisection.
//
// C++ port of sw-baseline-python/placer.py. Reads and writes the same JSON
// netlist format (see json_utils.py for the schema).

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <numeric>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "json.hpp"

using ordered_json = nlohmann::ordered_json;

// -- Constants ----------------------------------------------------------------

static constexpr int MAX_NET_DEGREE = 100;
static constexpr int DENSITY_BINS = 30;
static constexpr int MAX_OUTER_ITER = 15;
static constexpr int CG_MAX_ITER = 1000;
static constexpr double CG_EPS = 1e-5;
static constexpr double TARGET_DENSITY = 0.75;

// -- Data structures ----------------------------------------------------------

struct Macro {
    std::string name;
    double width;   // microns
    double height;  // microns
    std::vector<std::string> pins;
};

struct Component {
    std::string name;
    std::string macro_name;
    double x;  // database units
    double y;
};

struct IOPin {
    std::string name;
    std::string net_name;
    double x;  // database units
    double y;
};

struct Net {
    std::string name;
    std::vector<std::pair<std::string, std::string>> pins;  // (comp, pin)
};

struct Netlist {
    std::string design_name;
    int dbu_per_micron;
    double die_area[4];  // x1, y1, x2, y2

    std::unordered_map<std::string, Macro> macros;

    // Components stored in insertion order
    std::vector<std::string> cell_names;
    std::unordered_map<std::string, Component> components;
    std::unordered_map<std::string, int> cell_index;

    std::vector<IOPin> io_pins;
    std::unordered_map<std::string, const IOPin*> io_pin_map;

    std::vector<Net> nets;

    // Precomputed cell dimensions (database units)
    std::vector<double> cell_widths;
    std::vector<double> cell_heights;
    std::vector<double> cell_areas;

    int num_cells() const { return (int)cell_names.size(); }
};

// -- CSR sparse matrix --------------------------------------------------------

struct CSRMatrix {
    int n;  // dimension (n x n)
    std::vector<int> row_ptr;
    std::vector<int> col_idx;
    std::vector<double> vals;

    int nnz() const { return (int)vals.size(); }
};

#ifdef USE_HW_CG
#ifdef USE_FP_GOLDEN
#include "cg_fp_golden.h"
#else
#include "cg_hw_driver.h"
#endif
#endif
 
struct COOEntry {
    int row, col;
    double val;
};

CSRMatrix coo_to_csr(int n, std::vector<COOEntry>& entries) {
    // Sort by (row, col)
    std::sort(entries.begin(), entries.end(),
              [](const COOEntry& a, const COOEntry& b) {
                  return a.row < b.row || (a.row == b.row && a.col < b.col);
              });

    CSRMatrix m;
    m.n = n;
    m.row_ptr.resize(n + 1, 0);

    // Merge duplicates and build CSR
    for (size_t i = 0; i < entries.size();) {
        int r = entries[i].row, c = entries[i].col;
        double v = 0.0;
        while (i < entries.size() && entries[i].row == r && entries[i].col == c) {
            v += entries[i].val;
            ++i;
        }
        if (v != 0.0) {
            m.col_idx.push_back(c);
            m.vals.push_back(v);
            m.row_ptr[r + 1]++;
        }
    }

    // Prefix sum for row_ptr
    for (int i = 0; i < n; ++i) {
        m.row_ptr[i + 1] += m.row_ptr[i];
    }

    return m;
}

// Sparse matrix-vector multiply: y = A * x
std::vector<double> spmv(const CSRMatrix& A, const std::vector<double>& x) {
    std::vector<double> y(A.n, 0.0);
    for (int i = 0; i < A.n; ++i) {
        double s = 0.0;
        for (int j = A.row_ptr[i]; j < A.row_ptr[i + 1]; ++j) {
            s += A.vals[j] * x[A.col_idx[j]];
        }
        y[i] = s;
    }
    return y;
}

// Extract diagonal of CSR matrix
std::vector<double> csr_diagonal(const CSRMatrix& A) {
    std::vector<double> diag(A.n, 0.0);
    for (int i = 0; i < A.n; ++i) {
        for (int j = A.row_ptr[i]; j < A.row_ptr[i + 1]; ++j) {
            if (A.col_idx[j] == i) {
                diag[i] = A.vals[j];
                break;
            }
        }
    }
    return diag;
}

// Create a copy of A with alpha added to its diagonal
CSRMatrix csr_add_diagonal(const CSRMatrix& A, double alpha) {
    CSRMatrix B = A;  // copy
    for (int i = 0; i < B.n; ++i) {
        bool found = false;
        for (int j = B.row_ptr[i]; j < B.row_ptr[i + 1]; ++j) {
            if (B.col_idx[j] == i) {
                B.vals[j] += alpha;
                found = true;
                break;
            }
        }
        // If diagonal entry doesn't exist, we need to insert it.
        // For our use case, Q_base always has diagonal entries, so this
        // shouldn't happen, but handle it defensively.
        if (!found) {
            // Find insertion point
            int pos = B.row_ptr[i];
            while (pos < B.row_ptr[i + 1] && B.col_idx[pos] < i) ++pos;
            B.col_idx.insert(B.col_idx.begin() + pos, i);
            B.vals.insert(B.vals.begin() + pos, alpha);
            for (int k = i + 1; k <= B.n; ++k) B.row_ptr[k]++;
        }
    }
    return B;
}

// -- Vector helpers -----------------------------------------------------------

static double vec_dot(const std::vector<double>& a,
                      const std::vector<double>& b) {
    double s = 0.0;
    for (size_t i = 0; i < a.size(); ++i) s += a[i] * b[i];
    return s;
}

// -- Conjugate gradient solver ------------------------------------------------

// Solve Qx * x = -cx, starting from x0. Returns solution in x.
// Matches the CG algorithm from the project proposal (Listing 1):
//   x  = x0
//   r  = -cx - Qx * x
//   d  = r
//   rr = dot(r, r)
//   for k = 1, 2, ... do
//       q      = Qx * d
//       alpha  = rr / dot(d, q)
//       x      = x + alpha * d
//       r      = r - alpha * q
//       rr_new = dot(r, r)
//       if rr_new < eps^2 then return x
//       beta   = rr_new / rr
//       d      = r + beta * d
//       rr     = rr_new
//   end for
static void cg_solve(const CSRMatrix& Qx, const std::vector<double>& cx,
                     std::vector<double>& x, int max_iter, double eps) {
    int n = Qx.n;

    // r = -cx - Qx * x
    std::vector<double> Qx_x = spmv(Qx, x);
    std::vector<double> r(n), d(n);
    for (int i = 0; i < n; ++i) r[i] = -cx[i] - Qx_x[i];

    // d = r
    d = r;
    double rr = vec_dot(r, r);

    double eps2 = eps * eps;

    for (int k = 0; k < max_iter; ++k) {
        std::vector<double> q = spmv(Qx, d);
        double dq = vec_dot(d, q);
        if (dq == 0.0) break;
        double alpha = rr / dq;

        for (int i = 0; i < n; ++i) {
            x[i] += alpha * d[i];
            r[i] -= alpha * q[i];
        }

        double rr_new = vec_dot(r, r);
        if (rr_new < eps2) break;

        double beta = rr_new / rr;
        for (int i = 0; i < n; ++i) {
            d[i] = r[i] + beta * d[i];
        }
        rr = rr_new;
    }
}

// -- JSON I/O -----------------------------------------------------------------

Netlist load_netlist(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        std::fprintf(stderr, "Error: cannot open %s\n", path.c_str());
        std::exit(1);
    }
    ordered_json d = ordered_json::parse(f);

    Netlist nl;
    nl.design_name = d["design_name"].get<std::string>();
    nl.dbu_per_micron = d["dbu_per_micron"].get<int>();
    for (int i = 0; i < 4; ++i) nl.die_area[i] = d["die_area"][i].get<double>();

    // Macros
    for (auto& [name, m] : d["macros"].items()) {
        Macro macro;
        macro.name = name;
        macro.width = m["width"].get<double>();
        macro.height = m["height"].get<double>();
        for (auto& p : m["pins"]) macro.pins.push_back(p.get<std::string>());
        nl.macros[name] = std::move(macro);
    }

    // Components (preserve insertion order)
    for (auto& [name, c] : d["components"].items()) {
        Component comp;
        comp.name = name;
        comp.macro_name = c["macro_name"].get<std::string>();
        comp.x = c["x"].get<double>();
        comp.y = c["y"].get<double>();
        int idx = (int)nl.cell_names.size();
        nl.cell_names.push_back(name);
        nl.cell_index[name] = idx;
        nl.components[name] = std::move(comp);
    }

    // I/O pins
    for (auto& p : d["io_pins"]) {
        IOPin pin;
        pin.name = p["name"].get<std::string>();
        pin.net_name = p["net_name"].get<std::string>();
        pin.x = p["x"].get<double>();
        pin.y = p["y"].get<double>();
        nl.io_pins.push_back(std::move(pin));
    }
    for (auto& pin : nl.io_pins) {
        nl.io_pin_map[pin.name] = &pin;
    }

    // Nets
    for (auto& n : d["nets"]) {
        Net net;
        net.name = n["name"].get<std::string>();
        for (auto& pin : n["pins"]) {
            net.pins.emplace_back(pin[0].get<std::string>(),
                                  pin[1].get<std::string>());
        }
        nl.nets.push_back(std::move(net));
    }

    // Precompute cell dimensions
    int nc = nl.num_cells();
    nl.cell_widths.resize(nc, 0.0);
    nl.cell_heights.resize(nc, 0.0);
    nl.cell_areas.resize(nc, 0.0);
    for (int i = 0; i < nc; ++i) {
        auto it = nl.macros.find(nl.components[nl.cell_names[i]].macro_name);
        if (it != nl.macros.end()) {
            nl.cell_widths[i] = it->second.width * nl.dbu_per_micron;
            nl.cell_heights[i] = it->second.height * nl.dbu_per_micron;
            nl.cell_areas[i] = nl.cell_widths[i] * nl.cell_heights[i];
        }
    }

    return nl;
}

void dump_netlist(const Netlist& nl, const std::string& path) {
    ordered_json d;
    d["design_name"] = nl.design_name;
    d["dbu_per_micron"] = nl.dbu_per_micron;
    d["die_area"] = {nl.die_area[0], nl.die_area[1],
                     nl.die_area[2], nl.die_area[3]};

    // Macros
    ordered_json macros_json = ordered_json::object();
    for (auto& [name, m] : nl.macros) {
        ordered_json mj;
        mj["width"] = m.width;
        mj["height"] = m.height;
        mj["pins"] = m.pins;
        macros_json[name] = std::move(mj);
    }
    d["macros"] = std::move(macros_json);

    // Components (preserve insertion order via cell_names)
    ordered_json comps_json = ordered_json::object();
    for (auto& name : nl.cell_names) {
        auto& c = nl.components.at(name);
        ordered_json cj;
        cj["macro_name"] = c.macro_name;
        cj["x"] = c.x;
        cj["y"] = c.y;
        comps_json[name] = std::move(cj);
    }
    d["components"] = std::move(comps_json);

    // I/O pins
    ordered_json io_json = ordered_json::array();
    for (auto& p : nl.io_pins) {
        ordered_json pj;
        pj["name"] = p.name;
        pj["net_name"] = p.net_name;
        pj["x"] = p.x;
        pj["y"] = p.y;
        io_json.push_back(std::move(pj));
    }
    d["io_pins"] = std::move(io_json);

    // Nets
    ordered_json nets_json = ordered_json::array();
    for (auto& n : nl.nets) {
        ordered_json nj;
        nj["name"] = n.name;
        ordered_json pins_arr = ordered_json::array();
        for (auto& [comp, pin] : n.pins) {
            pins_arr.push_back({comp, pin});
        }
        nj["pins"] = std::move(pins_arr);
        nets_json.push_back(std::move(nj));
    }
    d["nets"] = std::move(nets_json);

    std::ofstream f(path);
    f << d.dump(2) << "\n";
}

// -- Placer -------------------------------------------------------------------

class Placer {
public:
    Netlist nl;

    // Cell center positions
    std::vector<double> x_pos, y_pos;

    // Base linear system (from clique decomposition)
    CSRMatrix Q_base;
    std::vector<double> c_base_x, c_base_y;

    // Active linear system (base + anchor springs)
    CSRMatrix Q;
    std::vector<double> c_x, c_y;

    int partition_level = 0;

#ifdef USE_HW_CG
    CGHwDriver hw_driver;
#endif

    explicit Placer(const std::string& json_path) : nl(load_netlist(json_path)) {}

    void init_cell_positions() {
        for (auto& [name, comp] : nl.components) {
            comp.x = 0;
            comp.y = 0;
        }
    }

    // -- Build connectivity matrix --------------------------------------------

    void build_system() {
        int n = nl.num_cells();

        // Initial positions (cell centers)
        x_pos.resize(n);
        y_pos.resize(n);
        for (int i = 0; i < n; ++i) {
            auto& comp = nl.components[nl.cell_names[i]];
            x_pos[i] = comp.x + nl.cell_widths[i] / 2;
            y_pos[i] = comp.y + nl.cell_heights[i] / 2;
        }

        // Build Q and c via clique decomposition
        std::vector<COOEntry> entries;
        c_base_x.assign(n, 0.0);
        c_base_y.assign(n, 0.0);

        int nets_used = 0;
        for (auto& net : nl.nets) {
            std::vector<int> movable;
            std::vector<double> fixed_x, fixed_y;
            std::unordered_map<std::string, bool> seen;

            for (auto& [comp_name, pin_name] : net.pins) {
                if (comp_name == "PIN") {
                    if (!seen.count(pin_name)) {
                        seen[pin_name] = true;
                        auto it = nl.io_pin_map.find(pin_name);
                        if (it != nl.io_pin_map.end()) {
                            fixed_x.push_back(it->second->x);
                            fixed_y.push_back(it->second->y);
                        }
                    }
                } else {
                    if (!seen.count(comp_name)) {
                        seen[comp_name] = true;
                        auto it = nl.cell_index.find(comp_name);
                        if (it != nl.cell_index.end()) {
                            movable.push_back(it->second);
                        }
                    }
                }
            }

            int p = (int)(movable.size() + fixed_x.size());
            if (p < 2 || p > MAX_NET_DEGREE) continue;
            nets_used++;

            double w = 2.0 / p;

            // Movable-movable clique edges
            for (size_t a = 0; a < movable.size(); ++a) {
                for (size_t b = a + 1; b < movable.size(); ++b) {
                    int i = movable[a], j = movable[b];
                    entries.push_back({i, i, w});
                    entries.push_back({j, j, w});
                    entries.push_back({i, j, -w});
                    entries.push_back({j, i, -w});
                }
            }

            // Movable-fixed edges
            for (int idx : movable) {
                for (size_t k = 0; k < fixed_x.size(); ++k) {
                    entries.push_back({idx, idx, w});
                    c_base_x[idx] -= w * fixed_x[k];
                    c_base_y[idx] -= w * fixed_y[k];
                }
            }
        }

        Q_base = coo_to_csr(n, entries);
        Q = Q_base;  // copy
        c_x = c_base_x;
        c_y = c_base_y;
        partition_level = 0;

        std::printf("  %d cells, %d I/O pins, %d/%d nets\n",
                    n, (int)nl.io_pins.size(), nets_used, (int)nl.nets.size());
        std::printf("  Q: %d nonzeros\n", Q_base.nnz());
    }

    // -- CG solver ------------------------------------------------------------

    void solve_cg() {
        // Solve Qx = -cx and Qy = -cy via CG (proposal Listing 1)
#ifdef USE_HW_CG
        if (Q.n <= CGHwDriver::MAX_N) {
            hw_driver.solve(Q, c_x, c_y, x_pos, y_pos, CG_MAX_ITER, CG_EPS);
        } else {
            fprintf(stderr, "Warning: n=%d > %d, falling back to software CG\n",
                    Q.n, CGHwDriver::MAX_N);
            cg_solve(Q, c_x, x_pos, CG_MAX_ITER, CG_EPS);
            cg_solve(Q, c_y, y_pos, CG_MAX_ITER, CG_EPS);
        }
#else
        cg_solve(Q, c_x, x_pos, CG_MAX_ITER, CG_EPS);
        cg_solve(Q, c_y, y_pos, CG_MAX_ITER, CG_EPS);
#endif
        clamp_to_die();
    }

    void clamp_to_die() {
        int n = nl.num_cells();
        double dx1 = nl.die_area[0], dy1 = nl.die_area[1];
        double dx2 = nl.die_area[2], dy2 = nl.die_area[3];
        for (int i = 0; i < n; ++i) {
            double hw = nl.cell_widths[i] / 2;
            double hh = nl.cell_heights[i] / 2;
            x_pos[i] = std::max(x_pos[i], dx1 + hw);
            x_pos[i] = std::min(x_pos[i], dx2 - hw);
            y_pos[i] = std::max(y_pos[i], dy1 + hh);
            y_pos[i] = std::min(y_pos[i], dy2 - hh);
        }
    }

    // -- Partition and anchor -------------------------------------------------

    void partition_and_anchor() {
        partition_level++;
        int n = nl.num_cells();
        double dx1 = nl.die_area[0], dy1 = nl.die_area[1];
        double dx2 = nl.die_area[2], dy2 = nl.die_area[3];

        // Region: (x1, y1, x2, y2, cell_indices)
        using Region = std::tuple<double, double, double, double,
                                  std::vector<int>>;
        std::vector<Region> regions;
        {
            std::vector<int> all(n);
            std::iota(all.begin(), all.end(), 0);
            regions.emplace_back(dx1, dy1, dx2, dy2, std::move(all));
        }

        for (int level = 0; level < partition_level; ++level) {
            std::vector<Region> new_regions;
            bool cut_x = (level % 2 == 0);

            for (auto& [rx1, ry1, rx2, ry2, indices] : regions) {
                if (indices.size() <= 1) {
                    new_regions.emplace_back(rx1, ry1, rx2, ry2,
                                             std::move(indices));
                    continue;
                }

                // Sort by position
                if (cut_x) {
                    std::sort(indices.begin(), indices.end(),
                              [&](int a, int b) {
                                  return x_pos[a] < x_pos[b];
                              });
                } else {
                    std::sort(indices.begin(), indices.end(),
                              [&](int a, int b) {
                                  return y_pos[a] < y_pos[b];
                              });
                }

                // Split at median area
                double half_area = 0.0;
                for (int i : indices) half_area += nl.cell_areas[i];
                half_area /= 2;

                double cumulative = 0.0;
                int mid = (int)indices.size() / 2;  // fallback
                for (int k = 0; k < (int)indices.size(); ++k) {
                    cumulative += nl.cell_areas[indices[k]];
                    if (cumulative >= half_area) {
                        mid = std::max(1, k + 1);
                        break;
                    }
                }

                std::vector<int> left(indices.begin(),
                                      indices.begin() + mid);
                std::vector<int> right(indices.begin() + mid,
                                       indices.end());

                if (cut_x) {
                    double mx = (rx1 + rx2) / 2;
                    new_regions.emplace_back(rx1, ry1, mx, ry2,
                                             std::move(left));
                    new_regions.emplace_back(mx, ry1, rx2, ry2,
                                             std::move(right));
                } else {
                    double my = (ry1 + ry2) / 2;
                    new_regions.emplace_back(rx1, ry1, rx2, my,
                                             std::move(left));
                    new_regions.emplace_back(rx1, my, rx2, ry2,
                                             std::move(right));
                }
            }
            regions = std::move(new_regions);
        }

        // Compute anchor points (region centers)
        std::vector<double> anchors_x(n, 0.0), anchors_y(n, 0.0);
        for (auto& [rx1, ry1, rx2, ry2, indices] : regions) {
            double cx = (rx1 + rx2) / 2;
            double cy = (ry1 + ry2) / 2;
            for (int i : indices) {
                anchors_x[i] = cx;
                anchors_y[i] = cy;
            }
        }

        // Anchor weight
        std::vector<double> diag = csr_diagonal(Q_base);
        double sum_pos = 0.0;
        int count_pos = 0;
        for (double d : diag) {
            if (d > 0.0) { sum_pos += d; count_pos++; }
        }
        double avg_diag = count_pos > 0 ? sum_pos / count_pos : 1.0;
        double alpha = avg_diag * 0.1 * std::pow(2.0, partition_level - 1);

        // Q = Q_base + alpha*I
        Q = csr_add_diagonal(Q_base, alpha);

        // c = c_base - alpha * anchors
        c_x.resize(n);
        c_y.resize(n);
        for (int i = 0; i < n; ++i) {
            c_x[i] = c_base_x[i] - alpha * anchors_x[i];
            c_y[i] = c_base_y[i] - alpha * anchors_y[i];
        }

        std::printf("  Partition level %d: %d regions, alpha=%.2f\n",
                    partition_level, (int)regions.size(), alpha);
    }

    // -- Check overlap --------------------------------------------------------

    double max_bin_density() {
        double dx1 = nl.die_area[0], dy1 = nl.die_area[1];
        double dx2 = nl.die_area[2], dy2 = nl.die_area[3];
        double die_w = dx2 - dx1, die_h = dy2 - dy1;
        double bin_w = die_w / DENSITY_BINS;
        double bin_h = die_h / DENSITY_BINS;
        double bin_area = bin_w * bin_h;

        std::vector<double> density(DENSITY_BINS * DENSITY_BINS, 0.0);

        for (int i = 0; i < nl.num_cells(); ++i) {
            double cx = x_pos[i], cy = y_pos[i];
            double w = nl.cell_widths[i], h = nl.cell_heights[i];

            double x1 = std::max(cx - w / 2, dx1);
            double y1 = std::max(cy - h / 2, dy1);
            double x2 = std::min(cx + w / 2, dx2);
            double y2 = std::min(cy + h / 2, dy2);

            int bx1 = std::max(0, (int)((x1 - dx1) / bin_w));
            int by1 = std::max(0, (int)((y1 - dy1) / bin_h));
            int bx2 = std::min(DENSITY_BINS - 1, (int)((x2 - dx1) / bin_w));
            int by2 = std::min(DENSITY_BINS - 1, (int)((y2 - dy1) / bin_h));

            for (int bx = bx1; bx <= bx2; ++bx) {
                for (int by = by1; by <= by2; ++by) {
                    double ox1 = std::max(x1, dx1 + bx * bin_w);
                    double oy1 = std::max(y1, dy1 + by * bin_h);
                    double ox2 = std::min(x2, dx1 + (bx + 1) * bin_w);
                    double oy2 = std::min(y2, dy1 + (by + 1) * bin_h);
                    double area = std::max(0.0, ox2 - ox1) *
                                  std::max(0.0, oy2 - oy1);
                    density[bx * DENSITY_BINS + by] += area;
                }
            }
        }

        double md = 0.0;
        for (double& d : density) {
            d /= bin_area;
            md = std::max(md, d);
        }
        return md;
    }

    bool check_overlap() {
        double md = max_bin_density();
        std::printf("  Max bin density: %.2f\n", md);
        return md <= TARGET_DENSITY;
    }

    // -- HPWL -----------------------------------------------------------------

    double compute_hpwl() {
        double total = 0.0;
        for (auto& net : nl.nets) {
            double min_x = std::numeric_limits<double>::infinity();
            double max_x = -std::numeric_limits<double>::infinity();
            double min_y = std::numeric_limits<double>::infinity();
            double max_y = -std::numeric_limits<double>::infinity();
            int count = 0;

            for (auto& [comp_name, pin_name] : net.pins) {
                if (comp_name == "PIN") {
                    auto it = nl.io_pin_map.find(pin_name);
                    if (it != nl.io_pin_map.end()) {
                        min_x = std::min(min_x, it->second->x);
                        max_x = std::max(max_x, it->second->x);
                        min_y = std::min(min_y, it->second->y);
                        max_y = std::max(max_y, it->second->y);
                        count++;
                    }
                } else {
                    auto it = nl.cell_index.find(comp_name);
                    if (it != nl.cell_index.end()) {
                        int idx = it->second;
                        min_x = std::min(min_x, x_pos[idx]);
                        max_x = std::max(max_x, x_pos[idx]);
                        min_y = std::min(min_y, y_pos[idx]);
                        max_y = std::max(max_y, y_pos[idx]);
                        count++;
                    }
                }
            }

            if (count >= 2) {
                total += (max_x - min_x) + (max_y - min_y);
            }
        }
        return total;
    }

    // -- Output ---------------------------------------------------------------

    void update_components() {
        for (int i = 0; i < nl.num_cells(); ++i) {
            auto& comp = nl.components[nl.cell_names[i]];
            comp.x = x_pos[i] - nl.cell_widths[i] / 2;
            comp.y = y_pos[i] - nl.cell_heights[i] / 2;
        }
    }

    std::string write_output(const std::string& tag = "") {
        std::string out_name = nl.design_name;
        if (!tag.empty()) out_name += "-" + tag;
        out_name += ".json";
        dump_netlist(nl, out_name);
        return out_name;
    }
};

// -- Entry point --------------------------------------------------------------

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::fprintf(stderr, "Usage: %s <netlist.json>\n", argv[0]);
        std::fprintf(stderr, "  e.g. %s DMA.json\n", argv[0]);
        return 1;
    }

    Placer placer(argv[1]);

    // Initialize cell positions and write initial JSON
    std::printf("Initializing cell positions...\n");
    placer.init_cell_positions();
    std::string out_path = placer.write_output("initial");
    std::printf("  Initial placement written to %s\n", out_path.c_str());

    // Step 1: Build connectivity matrix
    std::printf("Step 1: Building connectivity matrix...\n");
    placer.build_system();

    // Step 2: Initial CG solve
    std::printf("Step 2: Initial CG solve...\n");
    placer.solve_cg();
    std::printf("  HPWL: %.0f\n", placer.compute_hpwl());

    // Steps 3-4: Iterative spreading
    bool converged = false;
    double prev_density = placer.max_bin_density();
    for (int iter = 1; iter <= MAX_OUTER_ITER; ++iter) {
        std::printf("Iteration %d:\n", iter);
        placer.partition_and_anchor();

        // Save positions before CG solve
        auto x_saved = placer.x_pos;
        auto y_saved = placer.y_pos;

        placer.solve_cg();
        std::printf("  HPWL: %.0f\n", placer.compute_hpwl());

        double new_density = placer.max_bin_density();
        // Only revert once spreading has started working (density below
        // 2x target). Early iterations often increase density temporarily.
        if (prev_density < 2 * TARGET_DENSITY && new_density > prev_density) {
            std::printf("  Max bin density: %.2f (worse than %.2f, reverting)\n",
                        new_density, prev_density);
            placer.x_pos = x_saved;
            placer.y_pos = y_saved;
            break;
        }
        std::printf("  Max bin density: %.2f\n", new_density);
        prev_density = new_density;
        if (new_density <= TARGET_DENSITY) {
            std::printf("Overlap acceptable -- placement complete.\n");
            converged = true;
            break;
        }
    }
    if (!converged) {
        std::printf("Max iterations (%d) reached.\n", MAX_OUTER_ITER);
    }

    // Write final output
    placer.update_components();
    out_path = placer.write_output("final");
    std::printf("Final placement written to %s\n", out_path.c_str());

    return 0;
}

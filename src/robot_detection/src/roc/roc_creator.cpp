/// HEADER
#include "roc_creator.h"

/// SYSTEM
#include <fstream>
#include <yaml-cpp/yaml.h>

namespace
{
template <class T>
bool less_than(const T& a, const T& b)
{
    return (a.data < b.data);
}

int mx = -1;
int my = -1;
bool redraw_overlay = true;

enum EVENT {
    NONE,
    SAVE,
    SHOW_NEAREST_POSITIVE,
    SHOW_NEARERT_NEGATIVE
};

EVENT userevent = NONE;

void mouse(int event, int x, int y, int/* flags*/, void* /*param*/)
{
    if(x != mx || y != my) {
        mx = x;
        my = y;
        redraw_overlay = true;
    }

    if(event == CV_EVENT_LBUTTONUP) {
        userevent = SAVE;
    } else if(event == CV_EVENT_MBUTTONUP) {
        userevent = SHOW_NEAREST_POSITIVE;
    } else if(event == CV_EVENT_RBUTTONUP) {
        userevent = SHOW_NEARERT_NEGATIVE;
    }
}

}

RocCreator::RocCreator(int width, int padding, const std::string& title)
    : p(0), n(0),
      fp(0), tp(0), dimension_raw(width), w(width), padding(padding),
      keep_running(true),
      title(title),
      zoom(10), zoom_max(100)
{
    initImage();
}

void RocCreator::setTitle(const std::string& title)
{
    this->title = title;
}

void RocCreator::initImage()
{
    double factor = zoom / 10.0;
    w = dimension_raw * factor;
    roc = cv::Mat(w + 2 * padding, w + 2 * padding, CV_8UC3, cv::Scalar::all(255));
}

namespace YAML
{
Emitter& operator << (Emitter& emitter, const RocCreator::Data& d)
{
    emitter << YAML::BeginSeq;
    emitter << d.score;
    emitter << d.dtheta;
    emitter << d.dr;
    emitter << YAML::EndSeq;

    return emitter;
}

void operator >> (const Node& node, RocCreator::Data& d)
{
    try {
        node[0] >> d.score;
        node[1] >> d.dtheta;
        node[2] >> d.dr;
    } catch(YAML::InvalidScalar& e) {
    }
}

Emitter& operator << (Emitter& emitter, const RocCreator::L_data& pair)
{
    emitter << YAML::BeginSeq;
    emitter << pair.data;
    emitter << pair.positive;
    emitter << YAML::EndSeq;

    return emitter;
}

void operator >> (const Node& node, RocCreator::L_data& pair)
{
    try {
        node[0] >> pair.data;
    } catch(YAML::InvalidScalar& e) {
    }

    node[1] >> pair.positive;
}
}

void RocCreator::save(const std::string& path)
{
    // .png
    cv::imwrite(path, roc);

    // .roc
    std::string roc_path = path + ".roc";

    YAML::Emitter emitter;
    emitter << p;
    emitter << n;
    emitter << w;
    emitter << padding;
    emitter << L;
    std::ofstream fout(roc_path.c_str());
    fout << emitter.c_str();
    fout.close();

    // .csv
    std::string csv_path = path + ".csv";

    fout.open(csv_path.c_str());
    for(std::vector<R_data>::iterator pt = R.begin(); pt != R.end(); ++pt) {
        fout << pt->pt.x << "," << pt->pt.y << "\n";
    }
    fout.close();
}

bool RocCreator::load(const std::string& path)
{
    std::ifstream ifs(path.c_str());
    YAML::Parser parser(ifs);
    YAML::Node doc;

    if(!parser.GetNextDocument(doc)) {
        ERROR("cannot parse " << path);
        return false;
    }
    doc >> p;

    if(!parser.GetNextDocument(doc)) {
        ERROR("cannot parse " << path);
        return false;
    }
    doc >> n;

    if(!parser.GetNextDocument(doc)) {
        ERROR("cannot parse " << path);
        return false;
    }
    doc >> w;

    if(!parser.GetNextDocument(doc)) {
        ERROR("cannot parse " << path);
        return false;
    }
    doc >> padding;

    if(!parser.GetNextDocument(doc)) {
        ERROR("cannot parse " << path);
        return false;
    }
    doc >> L;

    for(std::vector<L_data>::const_iterator it = L.begin(); it != L.end(); ++it) {
        if(it->positive) {
            p++;
        } else {
            n++;
        }
    }

    initImage();
    draw();

    return true;
}

void RocCreator::getErrorsForThreshold(double threshold, std::vector<double> &out, double& tp, double& fp)
{
    out.clear();

    int real_tp = 0;
    int real_fp = 0;

    for(std::vector<L_data>::const_iterator it = L.begin(); it != L.end(); ++it) {
        if(it->data.score > threshold) {
            break;
        }


        bool is_positive = it->positive;
        if(is_positive) {
            out.push_back(it->data.dtheta);
            real_tp++;
        } else {
            real_fp++;
        }
    }

    tp = real_tp / ((double) p);
    fp = real_fp / ((double) n);
}

bool RocCreator::add(const Data& data, bool is_positive_example)
{
    L.push_back(L_data());
    L_data& d = L[L.size()-1];

    d.data = data;
    d.positive = is_positive_example;

    if(is_positive_example) {
        p++;
    } else {
        n++;
    }

    // sort L decreasing by probability -> increasing by score
    std::sort(L.begin(), L.end(), less_than<RocCreator::L_data>);

    return draw();
}

bool RocCreator::draw()
{
    initImage();

    fp = 0;
    tp = 0;

    R.clear();

    double f_pref = -INFINITY;

    for(std::vector<L_data>::const_iterator it = L.begin(); it != L.end(); ++it) {
        double f = it->data.score;
        assert(f >= f_pref);
        bool is_positive = it->positive;

        R.push_back(R_data());
        R_data& data = R[R.size()-1];
        data.positive = is_positive;
        data.data = &*it;
        data.pt.x = fp / (double) std::max(1, n);
        data.pt.y = tp / (double) std::max(1, p);

        if(f != f_pref) {
            data.meta = false;
            f_pref = f;

        } else {
            data.meta = true;
        }

        if(is_positive) {
            tp++;
        } else {
            fp++;
        }
    }

    R.push_back(R_data());
    R_data& data = R[R.size()-1];

    data.data = &L[L.size()-1];
    data.pt.x = fp / (double) n;
    data.pt.y = tp / (double) p;
    data.meta = true;

    cv::line(roc, cv::Point(padding, roc.rows - padding), cv::Point(roc.cols - padding, padding), cv::Scalar::all(128), 1, CV_AA);
    cv::line(roc, cv::Point(padding, roc.rows - padding), cv::Point(padding, padding), cv::Scalar::all(0), 2, CV_AA);
    cv::line(roc, cv::Point(padding, roc.rows - padding), cv::Point(roc.cols - padding, roc.rows - padding), cv::Scalar::all(0), 2, CV_AA);

    cv::Point last(padding, roc.rows - padding);
    std::vector<R_data>::iterator pred = R.begin();
    std::vector<R_data>::iterator last_non_meta = R.begin();
    std::vector<R_data>::iterator last_pt = R.end() - 1;
    int meta_count = -1;
    int meta_i = 0;

    for(std::vector<R_data>::iterator pt = R.begin(); pt != R.end(); ++pt) {
        cv::Point current;

        if(!pt->meta) {
            // if pt is not a meta point, no special things to do
            current = cv::Point(padding + pt->pt.x * w, roc.rows - (padding + pt->pt.y * w));
            meta_count = -1;
            meta_i = 0;

        } else {
            // if pt is a meta point -> find next non meta
            R_data* next = NULL;

            int mc = 0;
            for(std::vector<R_data>::iterator search = pt; search != R.end(); ++search) {
                mc++;
                if(!search->meta || search == last_pt) {
                    next = &*search;
                    break;
                }
            }

            if(meta_count == -1) {
                meta_count = mc;
            }

            assert(next != NULL);

            double dx = next->pt.x - last_non_meta->pt.x;
            double dy = next->pt.y - last_non_meta->pt.y;
            double factor = meta_i / (double) meta_count;
            double x = last_non_meta->pt.x + dx * factor;
            double y = last_non_meta->pt.y + dy * factor;

            pt->pt.x = x;
            pt->pt.y = y;

            current = cv::Point(padding + x * w, roc.rows - (padding + y * w));
            meta_i++;
        }

        cv::Scalar line_col = (pred->meta) ? cv::Scalar(127, 127, 127) : cv::Scalar(0, 0, 255);
        cv::line(roc, last, current, line_col, 1, CV_AA);

        cv::Scalar col = (pt->positive) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
        cv::circle(roc, current, 3, col, CV_FILLED, CV_AA);

        if(!pt->meta) {
            last_non_meta = pt;
        }

        last = current;
        pred = pt;
    }
    //    for(std::vector<std::pair<double, cv::Point2d> >::iterator pt = R.begin(); pt != R.end(); ++pt){
    //        cv::Point current(padding + pt->second.x * w, roc.rows - (padding + pt->second.y * w));
    //        cv::circle(roc, current, 3, cv::Scalar(0, 0, 255), 1, CV_AA);
    //    }

    cv::putText(roc, title, cv::Point(padding * 2, w + padding - 6), cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar::all(0), 1, CV_AA);

    cv::imshow("roc", roc);
    int key = (cv::waitKey(33) & 0xFF);

    return key != 27 && cvGetWindowHandle("roc") != 0;
}

void RocCreator::displayInteractive()
{
    try {
        cv::Mat display;

        cv::setMouseCallback("roc", &mouse);

        keep_running = true;
        const Data* best = NULL;

        int real_tp = 0;
        int real_fp = 0;

        while(keep_running) {
            if(redraw_overlay) {
                roc.copyTo(display);

                double dr = 0;
                double dtheta = 0;
                double best_dist = INFINITY;
                cv::Point2d closest(0, 0);

                for(std::vector<R_data>::iterator pt = R.begin(); pt != R.end(); ++pt) {
                    cv::Point current(padding + pt->pt.x * w, roc.rows - (padding + pt->pt.y * w));

                    double dx = current.x - mx;
                    double dy = current.y - my;

                    double dist = dx*dx + dy*dy;

                    if(dist < best_dist) {
//                        bool ok = true;
//                        if(userevent == SHOW_NEARERT_NEGATIVE || userevent == SHOW_NEAREST_POSITIVE){
//                            bool looking_for_positive = userevent == SHOW_NEAREST_POSITIVE;
//                            ok = pt->data->positive == looking_for_positive;
//                        }
//                        if(ok) {
                        best_dist = dist;
                        best = &(pt->data->data);
                        closest = current;
//                        }
                    }
                }

                real_tp = 0;
                real_fp = 0;
                for(std::vector<L_data>::const_iterator it = L.begin(); it != L.end(); ++it) {
                    double f = it->data.score;
                    if(f >= best->score) {
                        break;
                    }

                    dtheta += it->data.dtheta;
                    dr += it->data.dr;

                    bool is_positive = it->positive;
                    if(is_positive) {
                        real_tp++;
                    } else {
                        real_fp++;
                    }
                }

                redraw_overlay = false;

                cv::circle(display, closest, 3, cv::Scalar(255, 0, 0), 1, CV_AA);

                std::stringstream ss;
                ss << "threshold:" << best->score << ", tp: " << (100.0 / p * real_tp) << "%, fp: " << (100.0 / n * real_fp) << "%";

                cv::putText(display, ss.str(), cv::Point(4, 14), cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.4, cv::Scalar::all(0), 1, CV_AA);

                ss.str(std::string());
                ss << "dr: " << (dr / tp) << ", dtheta: " << (dtheta / tp);
                cv::putText(display, ss.str(), cv::Point(4, 30), cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.4, cv::Scalar::all(0), 1, CV_AA);
            }

            if(userevent != NONE) {
                score_selected(this, best, userevent == SAVE);
                userevent = NONE;
            }

            try {
                cv::imshow("roc", display);
            } catch(cv::Exception& e) {
                ERROR("displayin roc failed: " << e.msg);
            }

            bool redraw_underlying_image = false;

            char key = cv::waitKey(33);
            switch(key) {
            case 27:
                keep_running = false;
                break;

            case '+':
                zoom = std::min(zoom_max, zoom + 1);
                redraw_underlying_image = true;
                WARN("zoom=" << zoom);
                break;

            case '-':
                zoom = std::max(1, zoom - 1);
                redraw_underlying_image = true;
                WARN("zoom=" << zoom);
                break;

            default:
                break;
            }

            if(redraw_underlying_image) {
                draw();
                redraw_overlay = true;
            }

            keep_running &= cvGetWindowHandle("roc") != NULL;
        }
    } catch(cv::Exception& e) {
        ERROR("Error in ROC Creator: " << e.what());
    }
}

void RocCreator::stop()
{
    keep_running = false;
}

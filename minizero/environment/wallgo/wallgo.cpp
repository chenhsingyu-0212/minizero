#include "wallgo.h"
#include "configuration.h"
#include <cassert>
#include <iomanip>
#include <iostream>
#include <queue>
#include <sstream>
#include <unordered_set>
#include <cctype>
#include <cstdlib>

namespace minizero::env::wallgo {
using namespace minizero::utils;

int rotateActionId(int aid, utils::Rotation rot)
{
    if (rot == utils::Rotation::kRotationNone) return aid;

    const int N = kWallGoBoardSize;
    const int FM = kWallGoNumMovePatterns * kWallGoNumWallDirs; // 13*4=52

    auto rotPos = [&](int p) -> int {
        return utils::getPositionByRotating(rot, p, N);
    };
    auto rotDir = [&](int d) -> int { // 0:U,1:R,2:D,3:L
        static const int r90[4] = {1, 2, 3, 0};
        static const int r180[4] = {2, 3, 0, 1};
        static const int r270[4] = {3, 0, 1, 2};
        switch (rot) {
            case utils::Rotation::kRotation90: return r90[d];
            case utils::Rotation::kRotation180: return r180[d];
            case utils::Rotation::kRotation270: return r270[d];
            default: return d;
        }
    };

    if (aid >= kWallGoMovePolicySize) {
        int pos = aid - kWallGoMovePolicySize;
        return kWallGoMovePolicySize + rotPos(pos);
    } else {
        const int from = aid / FM;
        const int t = aid % FM;
        const int mv_idx = t / kWallGoNumWallDirs;
        const int dir = t % kWallGoNumWallDirs;

        int fr = from / N, fc = from % N;
        const auto& d = kMoveDelta.at(mv_idx); // {dr, dc}
        int tr = fr + d[0], tc = fc + d[1];
        int to = (tr >= 0 && tr < N && tc >= 0 && tc < N) ? (tr * N + tc) : from;

        int from_rot = rotPos(from);
        int to_rot = rotPos(to);

        int fr_r, fr_c, tr_r, tr_c;
        posToRc(from_rot, fr_r, fr_c);
        posToRc(to_rot, tr_r, tr_c);
        int dr2 = tr_r - fr_r, dc2 = tr_c - fr_c;

        int mv_idx_rot = -1;
        for (int i = 0; i < kWallGoNumMovePatterns; ++i) {
            const auto& dd = kMoveDelta.at(i);
            if (dd[0] == dr2 && dd[1] == dc2) {
                mv_idx_rot = i;
                break;
            }
        }
        if (mv_idx_rot < 0) mv_idx_rot = 0;

        int dir_rot = rotDir(dir);
        return from_rot * FM + mv_idx_rot * kWallGoNumWallDirs + dir_rot;
    }
}

// ─────────────────────────────── WallGoAction ───────────────────────────────

WallGoAction::WallGoAction(int action_id, Player player)
{
    this->player_ = player;
    this->action_id_ = action_id;
    parseAction(action_id_);
}

WallGoAction::WallGoAction(const std::vector<std::string>& action_string_args, int /*board_size*/)
{
    assert(action_string_args.size() == 2);
    assert(action_string_args[0].size() == 1);

    switch (action_string_args[0][0]) {
        case 'R': case 'r': player_ = Player::kPlayer1; break;
        case 'B': case 'b': player_ = Player::kPlayer2; break;
        default:            player_ = Player::kPlayerNone; break;
    }
    assert(static_cast<int>(player_) > 0 && static_cast<int>(player_) <= kWallGoNumPlayer);

    auto coordToPos = [](const std::string& coord) -> int {
        if (coord.size() < 2) return -1;
        char col = std::toupper(static_cast<unsigned char>(coord[0]));
        int  c   = col - 'A';
        int  r   = std::atoi(coord.substr(1).c_str()) - 1;
        if (r < 0 || r >= kWallGoBoardSize || c < 0 || c >= kWallGoBoardSize) return -1;
        return rcToPos(r, c);
    };

    const std::string& s = action_string_args[1];
    size_t arrow_pos = s.find("->");
    size_t colon_pos = s.find(":W");

    if (arrow_pos == std::string::npos) {
        // ---- setup: 例如 "C3"
        int p = coordToPos(s.substr(0, 2));
        if (p < 0) {
            action_id_ = kWallGoPolicySize;  // invalid
            parseAction(action_id_);
            return;
        }
        action_id_ = kWallGoMovePolicySize + p; // setup 索引
        parseAction(action_id_);
        return;
    }

    // ---- Play: "A2->A3:W1"
    if (colon_pos == std::string::npos || colon_pos + 2 >= s.size()) {
        action_id_ = kWallGoPolicySize;  // invalid
        parseAction(action_id_);
        return;
    }

    int from_pos = coordToPos(s.substr(0, 2));
    int to_pos   = coordToPos(s.substr(arrow_pos + 2, 2));
    int dir      = std::atoi(s.substr(colon_pos + 2).c_str());

    if (from_pos < 0 || to_pos < 0 || dir < 0 || dir >= kWallGoNumWallDirs) {
        action_id_ = kWallGoPolicySize;  // invalid
        parseAction(action_id_);
        return;
    }

    int fr, fc, tr, tc;
    posToRc(from_pos, fr, fc);
    posToRc(to_pos,   tr, tc);
    int dr = tr - fr, dc = tc - fc;

    int mv_idx = -1;
    for (int i = 0; i < kWallGoNumMovePatterns; ++i) {
        const auto& d = kMoveDelta.at(i); // {dr, dc}
        if (d[0] == dr && d[1] == dc) { mv_idx = i; break; }
    }
    if (mv_idx < 0) {
        action_id_ = kWallGoPolicySize;
        parseAction(action_id_);
        return;
    }

    const int FM = kWallGoNumMovePatterns * kWallGoNumWallDirs; // 13*4=52
    action_id_ = from_pos * FM + mv_idx * kWallGoNumWallDirs + dir;

    parseAction(action_id_);
}


std::string WallGoAction::toConsoleString() const
{
    std::ostringstream out;
    out << (player_ == Player::kPlayer1 ? 'R' : 'B') << " ";
    if (isSetup()) {
        int r, c;
        posToRc(to_, r, c);
        out << static_cast<char>('A' + c) << (r + 1);
    } else {
        int fr, fc, tr, tc;
        posToRc(from_, fr, fc);
        posToRc(to_, tr, tc);
        out << static_cast<char>('A' + fc) << (fr + 1) << "->" << static_cast<char>('A' + tc) << (tr + 1) << ":W" << wall_dir_;
    }
    return out.str();
}

void WallGoAction::parseAction(int action_id)
{
    if (action_id < kWallGoMovePolicySize) {
        const int FM = kWallGoNumMovePatterns * kWallGoNumWallDirs;
        int from_idx = action_id / FM; // 0..48
        int rem = action_id % FM;
        int mv_idx = rem / kWallGoNumWallDirs; // 0..12
        int dir = rem % kWallGoNumWallDirs;    // 0..3

        from_ = from_idx;
        int from_r, from_c;
        posToRc(from_idx, from_r, from_c);
        int to_r = from_r + kMoveDelta.at(mv_idx)[0];
        int to_c = from_c + kMoveDelta.at(mv_idx)[1];
        if (to_r < 0 || to_r >= kWallGoBoardSize || to_c < 0 || to_c >= kWallGoBoardSize) {
            is_out_of_board_ = true; // out of board
            to_ = -1;
            wall_dir_ = dir; // not -1 → Play
        } else {
            is_out_of_board_ = false;
            to_ = rcToPos(to_r, to_c);
            wall_dir_ = dir; // not -1 → Play
        }
    } else {
        int pos = action_id - kWallGoMovePolicySize; // 0..48
        from_ = to_ = pos;
        wall_dir_ = -1; // -1 → Setup
    }
}

// ─────────────────────────────── WallGoEnv ───────────────────────────────

void WallGoEnv::reset()
{
    // reset game state
    setTurn(Player::kPlayer1);
    actions_.clear();

    // clear pieces
    for (int p = 0; p < kWallGoNumPlayer; ++p) pieces_[p].reset();

    // reset walls
    for (int r = 0; r < kWallGoBoardSize; ++r)
        for (int c = 0; c < kWallGoBoardSize - 1; ++c)
            vwall_[r][c] = WallOwner::kNone;

    for (int r = 0; r < kWallGoBoardSize - 1; ++r)
        for (int c = 0; c < kWallGoBoardSize; ++c)
            hwall_[r][c] = WallOwner::kNone;

    // reset setup
    mode_ = (minizero::config::env_wallgo_init_rule == 2 ? SetupMode::kMode2 : SetupMode::kMode1);
    setup_step_ = 0;

    // mode2 reset pieces
    if (mode_ == SetupMode::kMode2) {
        pieces_[0].set(rcToPos(5, 1), true); // Red B6
        pieces_[0].set(rcToPos(1, 5), true); // Red F2
        pieces_[1].set(rcToPos(1, 1), true); // Blue B2
        pieces_[1].set(rcToPos(5, 5), true); // Blue F6
    }
}

bool WallGoEnv::act(const WallGoAction& action)
{
    if (!isLegalAction(action)){
        return false;
    }
    actions_.push_back(action);
    if (action.isSetup()) {
        pieces_[playerIdx(getTurn())].set(action.getTo(), true);
        ++setup_step_;
        // update turn
        if (inSetup(mode_, setup_step_, countBits(pieces_[0]), countBits(pieces_[1])))
            setTurn(setupWhoseTurn(mode_, (setup_step_)));
        else
            setTurn(Player::kPlayer1); // after setup, Red starts first
        return true;
    } else {
        movePiece(action.getFrom(), action.getTo(), playerIdx(getTurn()));
        buildWall(action.getTo(), action.getWallDir());
        setTurn(action.nextPlayer());
        return true;
    }
}

bool WallGoEnv::act(const std::vector<std::string>& args)
{
    return act(WallGoAction(args));
}

std::vector<WallGoAction> WallGoEnv::getLegalActions() const
{
    std::vector<WallGoAction> actions;

    for (int i = 0; i < getPolicySize(); ++i) {
        WallGoAction a(i, getTurn());
        if (isLegalAction(a)) {
            actions.push_back(a);
        }
    }

    return actions;
}

bool WallGoEnv::isLegalAction(const WallGoAction& action) const
{
    int me = playerIdx(getTurn());
    if (getTurn() != action.getPlayer()) return false;                                                               // not your turn
    if (action.isOutOfBoard()) return false;                                                                         // to out of board
    if (action.isSetup() != inSetup(mode_, setup_step_, countBits(pieces_[0]), countBits(pieces_[1]))) return false; // action type mismatch

    if (action.isSetup()) {
        if (action.getActionID() < kWallGoMovePolicySize || action.getActionID() >= kWallGoPolicySize) return false; // not a setup action
        int pos = action.getTo();
        if (pos < 0 || pos >= kWallGoBoardSize * kWallGoBoardSize) return false;
        if (!inSetup(mode_, setup_step_, countBits(pieces_[0]), countBits(pieces_[1]))) return false; // not in setup
        if (anyPieceAt(pos)) return false;                                                            // cell occupied
        return true;
    }

    // Play
    if (action.getActionID() >= kWallGoMovePolicySize || action.getActionID() < 0) return false; // not a play action
    int from_pos = action.getFrom();
    int to_pos = action.getTo();
    int dir = action.getWallDir();
    if (from_pos < 0 || from_pos >= kWallGoBoardSize * kWallGoBoardSize) return false;
    if (to_pos < 0 || to_pos >= kWallGoBoardSize * kWallGoBoardSize) return false;
    if (dir < 0 || dir >= kWallGoNumWallDirs) return false;
    if (!pieces_[me].test(from_pos)) return false; // from must be my piece
    if (!canMove(from_pos, to_pos)) return false;
    if (!canBuildWallAt(to_pos, dir)) return false;

    return true;
}

bool WallGoEnv::isTerminal() const
{
    int p1 = countBits(pieces_[0]), p2 = countBits(pieces_[1]);
    if (inSetup(mode_, setup_step_, p1, p2)) return false; // still in setup

    if (noSideHasLegalAction()) return true;

    const int N = kWallGoBoardSize;
    const int NN = N * N;
    std::vector<char> vis(NN, 0);

    for (int s = 0; s < NN; ++s) {
        if (vis[s]) continue;

        int red_in = 0, blue_in = 0;
        std::queue<int> q;
        q.push(s);
        vis[s] = 1;

        std::vector<int> nb;
        while (!q.empty()) {
            int u = q.front();
            q.pop();

            if (pieces_[0].test(u)) ++red_in;
            if (pieces_[1].test(u)) ++blue_in;

            oneStepNeighbors(u, nb);
            for (int v : nb) {
                if (!vis[v]) {
                    vis[v] = 1;
                    q.push(v);
                }
            }
        }

        if (red_in > 0 && blue_in > 0) {
            return false;
        }
    }

    return true;
}

float WallGoEnv::getEvalScore(bool is_resign /*= false*/) const
{
    Player eval = Player::kPlayerNone;

    if (is_resign) {
        eval = getNextPlayer(getTurn(), kWallGoNumPlayer);
    } else {
        ScorePack sp = evaluateScores();

        if (sp.total_red > sp.total_blue) {
            eval = Player::kPlayer1; // Red
        } else if (sp.total_red < sp.total_blue) {
            eval = Player::kPlayer2; // Blue
        } else {
            if (sp.max_region_red > sp.max_region_blue) {
                eval = Player::kPlayer1;
            } else if (sp.max_region_red < sp.max_region_blue) {
                eval = Player::kPlayer2;
            } else {
                // sort region scores and compare
                int len = std::min(sp.region_red.size(), sp.region_blue.size());
                for (int i = 0; i < len; ++i) {
                    if (sp.region_red[i] > sp.region_blue[i]) {
                        eval = Player::kPlayer1;
                        break;
                    } else if (sp.region_red[i] < sp.region_blue[i]) {
                        eval = Player::kPlayer2;
                        break;
                    }
                }
            }
        }
    }

    switch (eval) {
        case Player::kPlayer1: return 1.0f;  // Red win
        case Player::kPlayer2: return -1.0f; // Blue win
        default: return 0.0f;                // Draw
    }
}

std::vector<float> WallGoEnv::getFeatures(utils::Rotation rotation) const
{
    const int N = kWallGoBoardSize;
    const int NN = N * N;
    const int C = getNumInputChannels(); // 8
    std::vector<float> feat(C * NN, 0.0f);

    auto at = [&](int ch, int pos) -> float& {
        return feat[ch * NN + pos];
    };
    auto R = utils::reversed_rotation[static_cast<int>(rotation)];

    // Piece features
    for (int pos = 0; pos < NN; ++pos) {
        int src = getRotatePosition(pos, R);
        if (pieces_[0].test(src)) at(0, pos) = 1.0f;
        if (pieces_[1].test(src)) at(1, pos) = 1.0f;
    }

    // helper
    auto rightWallFromOriginal = [&](int r, int c, WallOwner& who) -> bool {
        if (c >= N - 1) return false;
        int pA_rot = r * N + c;
        int pB_rot = r * N + (c + 1);
        int pA = getRotatePosition(pA_rot, R);
        int pB = getRotatePosition(pB_rot, R);
        int Ar = pA / N, Ac = pA % N, Br = pB / N, Bc = pB % N;

        if (Ar == Br && std::abs(Ac - Bc) == 1) {
            int cc = std::min(Ac, Bc);
            if (cc < 0 || cc >= N - 1) return false;
            who = vwall_[Ar][cc];
            return who != WallOwner::kNone;
        }
        return false;
    };

    // helpper
    auto bottomWallFromOriginal = [&](int r, int c, WallOwner& who) -> bool {
        if (r >= N - 1) return false;
        int pA_rot = r * N + c;
        int pB_rot = (r + 1) * N + c;
        int pA = getRotatePosition(pA_rot, R);
        int pB = getRotatePosition(pB_rot, R);
        int Ar = pA / N, Ac = pA % N, Br = pB / N, Bc = pB % N;

        if (Ac == Bc && std::abs(Ar - Br) == 1) {
            int rr = std::min(Ar, Br);
            who = hwall_[rr][Ac];
            return who != WallOwner::kNone;
        }
        return false;
    };

    // Wall features
    for (int r = 0; r < N; ++r) {
        for (int c = 0; c < N; ++c) {
            WallOwner who;
            // VWall → ch2/ch3
            if (rightWallFromOriginal(r, c, who)) {
                int pos = r * N + c;
                if (who == WallOwner::kRed) at(2, pos) = 1.0f;
                if (who == WallOwner::kBlue) at(3, pos) = 1.0f;
            }
            // HWall → ch4/ch5
            if (bottomWallFromOriginal(r, c, who)) {
                int pos = r * N + c;
                if (who == WallOwner::kRed) at(4, pos) = 1.0f;
                if (who == WallOwner::kBlue) at(5, pos) = 1.0f;
            }
        }
    }

    // ch6/ch7: who's turn
    const int base6 = 6 * NN, base7 = 7 * NN;
    if (getTurn() == Player::kPlayer1)
        std::fill(feat.begin() + base6, feat.begin() + base6 + NN, 1.0f);
    else
        std::fill(feat.begin() + base7, feat.begin() + base7 + NN, 1.0f);

    return feat;
}

std::vector<float> WallGoEnv::getActionFeatures(
    const WallGoAction& action, utils::Rotation rotation) const
{
    const int A = kWallGoPolicySize;
    std::vector<float> feat(A, 0.0f);
    int aid = action.getActionID();
    if (aid < 0 || aid >= A) return feat;

    int rid = (rotation == utils::Rotation::kRotationNone)
                  ? aid
                  : minizero::env::wallgo::rotateActionId(aid, rotation);
    if (rid >= 0 && rid < A) {
        feat[rid] = 1.0f;
    }

    return feat;
}

std::string WallGoEnv::toString() const
{
    const int N = kWallGoBoardSize; // 7

    const std::string WHITE = "\033[47m";  // 白底
    const std::string REDFG = "\033[31m";  // 紅字
    const std::string BLUEFG = "\033[34m"; // 藍字
    const std::string GRAY = "\033[97m";   // 灰字
    const std::string BLACK = "\033[30m";  // 黑字
    const std::string RESET = "\033[0m";

    // outside border
    auto UL = WHITE + BLACK + "┏" + RESET;
    auto UR = WHITE + BLACK + "┓" + RESET;
    auto BL = WHITE + BLACK + "┗" + RESET;
    auto BR = WHITE + BLACK + "┛" + RESET;
    auto HH = WHITE + BLACK + "━" + RESET;
    auto VV = WHITE + BLACK + "┃" + RESET;

    // inside elements
    auto GH = WHITE + GRAY + "━━━" + RESET;
    auto GV = WHITE + GRAY + "┃" + RESET;
    auto RH = WHITE + REDFG + "━━━" + RESET;
    auto RV = WHITE + REDFG + "┃" + RESET;
    auto BH = WHITE + BLUEFG + "━━━" + RESET;
    auto BV = WHITE + BLUEFG + "┃" + RESET;
    auto RD = WHITE + REDFG + " ● " + RESET;
    auto BD = WHITE + BLUEFG + " ● " + RESET;
    auto EM = WHITE + "   " + RESET;
    auto EX = WHITE + " " + RESET;

    auto pieceGlyph = [&](int pos) -> const std::string& {
        if (pieces_[0].test(pos)) return RD;
        if (pieces_[1].test(pos)) return BD;
        return EM;
    };

    auto vwallGlyph = [&](int r, int c) -> const std::string& {
        if (vwall_[r][c] == WallOwner::kRed) return RV;
        if (vwall_[r][c] == WallOwner::kBlue) return BV;
        return GV;
    };

    auto hwallGlyph = [&](int r, int c) -> const std::string& {
        if (hwall_[r][c] == WallOwner::kRed) return RH;
        if (hwall_[r][c] == WallOwner::kBlue) return BH;
        return GH;
    };

    std::ostringstream out;

    out << EX << EX << EX;
    for (int c = 0; c < N; ++c) out << EX << EX << WHITE + "\033[1;30m" + static_cast<char>('A' + c) + RESET << EX;
    out << EX << "\n";

    out << EX << EX << EX;
    out << UL;
    for (int c = 0; c < N + 20; ++c) out << HH;
    out << UR << "\n";

    for (int r = 0; r < N; ++r) {
        if (r != 0) {
            out << EX << EX << EX;
            out << VV;
            for (int c = 0; c < N; ++c) {
                out << hwallGlyph(r - 1, c);
                if (c < N - 1) {
                    out << EX;
                }
            }
            out << VV << "\n";
        }

        out << EX << WHITE + "\033[1;30m" + std::to_string(r + 1) + RESET << EX;
        out << VV;
        for (int c = 0; c < N; ++c) {
            int pos = r * N + c;
            out << pieceGlyph(pos);
            if (c < N - 1) {
                out << vwallGlyph(r, c);
            }
        }
        out << VV << "\n";
    }

    out << EX << EX << EX;
    out << BL;
    for (int c = 0; c < N + 20; ++c) out << HH;
    out << BR;

    out << "\n"
        << "Turn: " << (getTurn() == Player::kPlayer1 ? "Red" : "Blue") << "\n";
    out << "Setup: " << (inSetup(mode_, setup_step_, countBits(pieces_[0]), countBits(pieces_[1])) ? "Yes" : "No") << "\n";
    float eval = getEvalScore();
    out << "isTerminal: " << (isTerminal() ? std::string("Yes, ") + (eval == 0.0f ? "Draw" : (eval > 0.0f ? "Winner Red" : "Winner Blue")) : "No") << "\n";
    ScorePack sp = evaluateScores();
    out << "Score: Red " << sp.total_red << " (region ";
    for (size_t i = 0; i < sp.region_red.size(); ++i) {
        if (i > 0) out << ",";
        out << sp.region_red[i];
    }
    out << "), Blue " << sp.total_blue << " (region ";
    for (size_t i = 0; i < sp.region_blue.size(); ++i) {
        if (i > 0) out << ",";
        out << sp.region_blue[i];
    }
    out << ")\n\n";

    return out.str();
}

bool WallGoEnv::canMove(int from_pos, int to_pos) const
{
    if (from_pos == to_pos) return true;
    if (anyPieceAt(to_pos)) return false;

    int fr, fc, tr, tc;
    posToRc(from_pos, fr, fc);
    posToRc(to_pos, tr, tc);
    if (std::abs(fr - tr) + std::abs(fc - tc) > 2) return false;

    std::vector<int> reach;
    reachable2Steps(from_pos, reach);
    return std::find(reach.begin(), reach.end(), to_pos) != reach.end();
}

bool WallGoEnv::noSideHasLegalAction() const
{
    int p1 = countBits(pieces_[0]), p2 = countBits(pieces_[1]);
    if (inSetup(mode_, setup_step_, p1, p2)) return false;

    const int NN = kWallGoBoardSize * kWallGoBoardSize;
    auto hasLegalFor = [&](int me) {
        for (int from = 0; from < NN; ++from) {
            if (pieces_[me].test(from)) {
                std::vector<int> outs;
                reachable2Steps(from, outs);
                for (int to : outs) {
                    if (!canMove(from, to)) continue;
                    for (int dir = 0; dir < kWallGoNumWallDirs; ++dir)
                        if (canBuildWallAt(to, dir)) return true;
                }
            }
        }
        return false;
    };
    return !hasLegalFor(0) && !hasLegalFor(1);
}

void WallGoEnv::movePiece(int from_pos, int to_pos, int me)
{
    if (from_pos == to_pos) return;
    pieces_[me].reset(from_pos);
    pieces_[me].set(to_pos);
}

void WallGoEnv::buildWall(int pos, int dir)
{
    int r, c;
    posToRc(pos, r, c);
    auto w = kWallDir[dir];
    int rr = r + w.rOff, cc = c + w.cOff;

    WallOwner who = (turn_ == Player::kPlayer1 ? WallOwner::kRed : WallOwner::kBlue);
    if (w.horizontal) {
        setHWall(rr, cc, who);
    } else {
        setVWall(rr, cc, who);
    }
}

ScorePack WallGoEnv::evaluateScores() const
{
    ScorePack sp;
    const int N = kWallGoBoardSize;
    const int NN = N * N;

    std::vector<char> vis(NN, 0);
    struct Comp {
        std::vector<int> cells;
        int red = 0, blue = 0;
    };
    std::vector<Comp> comps;

    std::vector<int> nb;
    for (int s = 0; s < NN; ++s) {
        if (vis[s]) continue;

        Comp comp;
        std::queue<int> q;
        q.push(s);
        vis[s] = 1;

        while (!q.empty()) {
            int u = q.front();
            q.pop();
            comp.cells.push_back(u);
            if (pieces_[0].test(u)) ++comp.red;
            if (pieces_[1].test(u)) ++comp.blue;

            oneStepNeighbors(u, nb);
            for (int v : nb) {
                if (!vis[v]) {
                    vis[v] = 1;
                    q.push(v);
                }
            }
        }
        comps.push_back(std::move(comp));
    }

    int total_red = 0, total_blue = 0;
    int max_region_red = 0, max_region_blue = 0;

    for (const auto& comp : comps) {
        if ((comp.red == 0 && comp.blue == 0) || (comp.red > 0 && comp.blue > 0)) continue;

        std::vector<char> in_comp(NN, 0);
        for (int p : comp.cells) {
            in_comp[p] = 1;
        }

        int side = (comp.red > 0 ? 0 : 1);

        std::queue<int> q;
        std::vector<char> vis2(NN, 0);
        for (int p : comp.cells) {
            if (pieces_[side].test(p)) {
                vis2[p] = 1;
                q.push(p);
            }
        }

        int reach_cnt = 0;
        while (!q.empty()) {
            int u = q.front();
            q.pop();
            ++reach_cnt;

            oneStepNeighbors(u, nb);
            for (int v : nb) {
                if (!in_comp[v] || vis2[v]) continue;
                if (anyPieceAt(v)) continue;
                vis2[v] = 1;
                q.push(v);
            }
        }

        if (side == 0) {
            total_red += reach_cnt;
            sp.region_red.push_back(reach_cnt);
            if (reach_cnt > max_region_red) max_region_red = reach_cnt;
        } else {
            total_blue += reach_cnt;
            sp.region_blue.push_back(reach_cnt);
            if (reach_cnt > max_region_blue) max_region_blue = reach_cnt;
        }
    }

    std::sort(sp.region_red.begin(), sp.region_red.end(), std::greater<int>());
    std::sort(sp.region_blue.begin(), sp.region_blue.end(), std::greater<int>());

    sp.total_red = total_red;
    sp.total_blue = total_blue;
    sp.max_region_red = max_region_red;
    sp.max_region_blue = max_region_blue;
    return sp;
}

// ─────────────────────────────── WallGoEnvLoader ───────────────────────────────

std::vector<float> WallGoEnvLoader::getActionFeatures(const int pos, utils::Rotation rotation) const
{
    const int A = kWallGoPolicySize;
    std::vector<float> feat(A, 0.0f);
    if (pos < 0 || pos >= A) return feat;

    int rid = minizero::env::wallgo::rotateActionId(pos, rotation);
    if (rid >= 0 && rid < A) feat[rid] = 1.0f;
    return feat;
}

} // namespace minizero::env::wallgo
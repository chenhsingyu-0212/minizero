#pragma once

#include "base_env.h"
#include <array>
#include <bitset>
#include <iostream>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace minizero::env::wallgo {

using namespace minizero::utils;
const std::string kWallGoName = "wallgo";
const int kWallGoNumPlayer = 2;
const int kWallGoBoardSize = 7;

// piece and wall
const int kWallGoNumPiecesPerPlayer = 4;
const int kWallGoPieces = kWallGoBoardSize * kWallGoBoardSize;                // 49
const int kWallGoHorizontalWalls = (kWallGoBoardSize - 1) * kWallGoBoardSize; // 42
const int kWallGoVerticalWalls = kWallGoBoardSize * (kWallGoBoardSize - 1);   // 42
const int kWallGoTotalWalls = kWallGoHorizontalWalls + kWallGoVerticalWalls;  // 84

// move patterns
const int kWallGoNumMovePatterns = 13;

// wall directions
const int kWallGoNumWallDirs = 4;

// action space: from(49) × move(13) × wall(4)  +  setup(49)
const int kWallGoMovePolicySize = kWallGoBoardSize * kWallGoBoardSize * kWallGoNumMovePatterns * kWallGoNumWallDirs; // 2548
const int kWallGoSetupPolicySize = kWallGoBoardSize * kWallGoBoardSize;                                              //   49
const int kWallGoPolicySize = kWallGoMovePolicySize + kWallGoSetupPolicySize;

const std::map<int, std::vector<int>> kMoveDelta = {
    {0, {0, 0}}, {1, {-1, 0}}, {2, {1, 0}}, {3, {0, -1}}, {4, {0, 1}}, {5, {-2, 0}}, {6, {2, 0}}, {7, {0, -2}}, {8, {0, 2}}, {9, {-1, -1}}, {10, {-1, 1}}, {11, {1, -1}}, {12, {1, 1}}};

struct ScorePack {
    int total_red = 0;
    int total_blue = 0;
    int max_region_red = 0;
    int max_region_blue = 0;
    std::vector<int> region_red;
    std::vector<int> region_blue;
};

struct WallMap {
    int dr, dc;
    bool horizontal;
    int rOff, cOff;
};

// up, right, down, left
const WallMap kWallDir[4] = {
    {-1, 0, true, -1, 0}, // up: hwall[r-1][c]
    {0, 1, false, 0, 0},  // right: vwall[r][c]
    {1, 0, true, 0, 0},   // down: hwall[r][c]
    {0, -1, false, 0, -1} // left: vwall[r][c-1]
};

// Setup modes
enum class SetupMode { kMode1,
                       kMode2 };

// helper
inline int rcToPos(int r, int c)
{
    return r * kWallGoBoardSize + c;
}
inline void posToRc(int pos, int& r, int& c)
{
    r = pos / kWallGoBoardSize;
    c = pos % kWallGoBoardSize;
}
inline int playerIdx(Player p) { return (p == Player::kPlayer1 ? 0 : 1); }
int rotateActionId(int aid, utils::Rotation rot);

class WallGoAction : public BaseBoardAction<kWallGoNumPlayer> {
public:
    WallGoAction()
    {
        player_ = Player::kPlayerNone;
        action_id_ = kWallGoMovePolicySize;
        from_ = to_ = 0;
        wall_dir_ = -1;
        is_out_of_board_ = false;
    }

    WallGoAction(int action_id, Player player);
    WallGoAction(const std::vector<std::string>& action_string_args, int board_size = minizero::config::env_board_size);
    Player nextPlayer() const override { return getNextPlayer(getPlayer(), kWallGoNumPlayer); }
    std::string toConsoleString() const override;
    void parseAction(int action_id);
    inline int getFrom() const { return from_; }
    inline int getTo() const { return to_; }
    inline int getWallDir() const { return wall_dir_; }
    inline bool isSetup() const { return wall_dir_ == -1; }
    inline bool isOutOfBoard() const { return is_out_of_board_; }

private:
    int from_ = 0;
    int to_ = 0;
    int wall_dir_ = -1;
    bool is_out_of_board_ = false;
};

class WallGoEnv : public BaseBoardEnv<WallGoAction> {
public:
    WallGoEnv() : BaseBoardEnv<WallGoAction>(kWallGoBoardSize) { reset(); }

    void reset() override;
    bool act(const WallGoAction& action) override;
    bool act(const std::vector<std::string>& action_string_args) override;
    std::vector<WallGoAction> getLegalActions() const override;
    bool isLegalAction(const WallGoAction& action) const override;
    bool isTerminal() const override;
    float getReward() const override { return 0.0f; }
    float getEvalScore(bool is_resign = false) const override;
    std::vector<float> getFeatures(utils::Rotation rotation = utils::Rotation::kRotationNone) const override;
    std::vector<float> getActionFeatures(const WallGoAction& action, utils::Rotation rotation = utils::Rotation::kRotationNone) const override;
    std::string toString() const override;
    inline int getNumInputChannels() const override { return 8; }
    inline int getNumActionFeatureChannels() const override
    {
        return kWallGoPolicySize / (kWallGoBoardSize * kWallGoBoardSize); // 2597 / 49 = 53
    }
    inline int getPolicySize() const override { return kWallGoPolicySize; }
    inline std::string name() const override { return kWallGoName; }
    inline int getNumPlayer() const override { return kWallGoNumPlayer; }
    inline int getRotatePosition(int position, utils::Rotation rotation) const override { return utils::getPositionByRotating(rotation, position, getBoardSize()); };
    inline int getRotateAction(int action_id, utils::Rotation rotation) const override { return minizero::env::wallgo::rotateActionId(action_id, rotation); };

    private:
        // pieces_[0] for Player1, pieces_[1] for Player2
        std::array<std::bitset<kWallGoPieces>, kWallGoNumPlayer> pieces_;
        // walls
        enum class WallOwner : int8_t { kNone = -1,
                                        kRed = 0,
                                        kBlue = 1 };
        std::array<std::array<WallOwner, kWallGoBoardSize - 1>, kWallGoBoardSize> vwall_; // vertical walls
        std::array<std::array<WallOwner, kWallGoBoardSize>, kWallGoBoardSize - 1> hwall_; // horizontal walls
        // setup state
        SetupMode mode_ = SetupMode::kMode2;
        int setup_step_ = 0;
        // helper functions
        bool anyPieceAt(int pos) const { return pieces_[0].test(pos) || pieces_[1].test(pos); }
        bool canMove(int from_pos, int to_pos) const;
        bool noSideHasLegalAction() const;
        void movePiece(int from_pos, int to_pos, int me);
        void buildWall(int pos, int dir);
        ScorePack evaluateScores() const;
        // mode helpers
        inline bool inBounds(int r, int c) const { return r >= 0 && r < kWallGoBoardSize && c >= 0 && c < kWallGoBoardSize; }
        inline bool vwallExists(int r, int c) const { return vwall_[r][c] != WallOwner::kNone; }
        inline bool hwallExists(int r, int c) const { return hwall_[r][c] != WallOwner::kNone; }
        inline WallOwner vwallOwner(int r, int c) const { return vwall_[r][c]; }
        inline WallOwner hwallOwner(int r, int c) const { return hwall_[r][c]; }
        inline void setVWall(int r, int c, WallOwner who) { vwall_[r][c] = who; }
        inline void setHWall(int r, int c, WallOwner who) { hwall_[r][c] = who; }
        inline int countBits(const std::bitset<kWallGoPieces>& b) const { return static_cast<int>(b.count()); }
        inline bool canBuildWallAt(int pos, int dir) const
        {
            int r, c;
            posToRc(pos, r, c);
            auto w = kWallDir[dir];
            int rr = r + w.rOff, cc = c + w.cOff;

            if (w.horizontal) {
                if (rr < 0 || rr >= kWallGoBoardSize - 1) return false;
                if (cc < 0 || cc >= kWallGoBoardSize) return false;
                return !hwallExists(rr, cc);
            } else {
                if (rr < 0 || rr >= kWallGoBoardSize) return false;
                if (cc < 0 || cc >= kWallGoBoardSize - 1) return false;
                return !vwallExists(rr, cc);
            }
        }
        inline static int ownerToIdx(WallOwner w)
        {
            if (w == WallOwner::kRed) return 0;
            if (w == WallOwner::kBlue) return 1;
            return -1; // or assert/throw if ever used on None
        }
        inline static bool inSetup(SetupMode mode, int setup_step, int p1, int p2)
        {
            if (mode == SetupMode::kMode1)
                return setup_step < 8 || p1 < 4 || p2 < 4;
            else
                return setup_step < 4 || p1 < 4 || p2 < 4;
        }
        inline static Player setupWhoseTurn(SetupMode mode, int setup_step)
        {
            if (mode == SetupMode::kMode1) {
                static const Player script[8] = {
                    Player::kPlayer1, Player::kPlayer2, Player::kPlayer2, Player::kPlayer1,
                    Player::kPlayer1, Player::kPlayer2, Player::kPlayer2, Player::kPlayer1};
                return script[setup_step];
            } else {
                if (setup_step == 1 || setup_step == 2)
                    return Player::kPlayer2;
                else
                    return Player::kPlayer1;
            }
        }
        inline bool edgeBlockedByWall(int a, int b) const
        {
            int ar, ac, br, bc;
            posToRc(a, ar, ac);
            posToRc(b, br, bc);
            if (std::abs(ar - br) + std::abs(ac - bc) != 1) return true;
            if (ar == br) {
                int c = std::min(ac, bc);
                return vwallExists(ar, c);
            }
            if (ac == bc) {
                int r = std::min(ar, br);
                return hwallExists(r, ac);
            }
            return true;
        }
        inline void oneStepNeighbors(int p, std::vector<int>& out) const
        {
            out.clear();
            int r = p / kWallGoBoardSize, c = p % kWallGoBoardSize;
            auto push = [&](int nr, int nc) {
                if (!inBounds(nr, nc)) return;
                int q = rcToPos(nr, nc);
                if (!edgeBlockedByWall(p, q)) out.push_back(q);
            };
            push(r + 1, c);
            push(r - 1, c);
            push(r, c + 1);
            push(r, c - 1);
        }
        inline void reachable2Steps(int from, std::vector<int>& outs) const
        {
            outs.clear();
            std::vector<char> seen(kWallGoBoardSize * kWallGoBoardSize, 0);

            auto push = [&](int x) { if (!seen[x]) { seen[x] = 1; outs.push_back(x); } };

            push(from);
            std::vector<int> n1;
            oneStepNeighbors(from, n1);
            for (int v : n1) push(v);
            for (int mid : n1) {
                if (anyPieceAt(mid)) continue;
                std::vector<int> n2;
                oneStepNeighbors(mid, n2);
                for (int dst : n2)
                    if (dst != from) push(dst);
            }
        }
    };

    // ───────────────────────────── Loader 類別 ─────────────────────────────
    class WallGoEnvLoader : public BaseBoardEnvLoader<WallGoAction, WallGoEnv> {
    public:
        std::vector<float> getActionFeatures(const int pos, utils::Rotation rotation = utils::Rotation::kRotationNone) const override;
        inline std::vector<float> getValue(const int pos) const { return {getReturn()}; }
        inline std::string name() const override { return kWallGoName; }
        inline int getPolicySize() const override { return kWallGoPolicySize; }
        inline int getRotatePosition(int position, utils::Rotation rotation) const override { return utils::getPositionByRotating(rotation, position, getBoardSize()); };
        inline int getRotateAction(int action_id, utils::Rotation rotation) const override { return minizero::env::wallgo::rotateActionId(action_id, rotation); };
    };
} // namespace minizero::env::wallgo
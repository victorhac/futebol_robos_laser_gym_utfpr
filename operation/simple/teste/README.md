protoc --proto_path=./proto --python_out=. ./proto/*.proto
docker compose --env-file .env up
https://cbr.robocup.org.br/wp-content/uploads/2024/08/sslrules.pdf

// All robots should completely stop moving.
HALT = 0;
// Robots must keep 50 cm from the ball.
STOP = 1;
// A prepared kickoff or penalty may now be taken.
NORMAL_START = 2;
// The ball is dropped and free for either team.
FORCE_START = 3;
// The yellow team may move into kickoff position.
PREPARE_KICKOFF_YELLOW = 4;
// The blue team may move into kickoff position.
PREPARE_KICKOFF_BLUE = 5;
// The yellow team may move into penalty position.
PREPARE_PENALTY_YELLOW = 6;
// The blue team may move into penalty position.
PREPARE_PENALTY_BLUE = 7;
// The yellow team may take a direct free kick.
DIRECT_FREE_YELLOW = 8;
// The blue team may take a direct free kick.
DIRECT_FREE_BLUE = 9;
// The yellow team may take an indirect free kick.
INDIRECT_FREE_YELLOW = 10 [deprecated = true];
// The blue team may take an indirect free kick.
INDIRECT_FREE_BLUE = 11 [deprecated = true];
// The yellow team is currently in a timeout.
TIMEOUT_YELLOW = 12;
// The blue team is currently in a timeout.
TIMEOUT_BLUE = 13;
// The yellow team just scored a goal.
// For information only.
// Deprecated: Use the score field from the team infos instead. That way, you can also detect revoked goals.
GOAL_YELLOW = 14 [deprecated = true];
// The blue team just scored a goal. See also GOAL_YELLOW.
GOAL_BLUE = 15 [deprecated = true];
// Equivalent to STOP, but the yellow team must pick up the ball and
// drop it in the Designated Position.
BALL_PLACEMENT_YELLOW = 16;
// Equivalent to STOP, but the blue team must pick up the ball and drop
// it in the Designated Position.
BALL_PLACEMENT_BLUE = 17;
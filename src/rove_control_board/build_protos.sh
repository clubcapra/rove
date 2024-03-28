rm -rf src/rove_control_board/api/cpp/*
rm -rf src/rove_control_board/api/python/*

protoc --proto_path=./src/rove_control_board/protos --cpp_out=./src/rove_control_board/api/cpp --python_out=./src/rove_control_board/api/python ./src/rove_control_board/protos/*.proto
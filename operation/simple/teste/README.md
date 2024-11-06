protoc --proto_path=./proto --python_out=. ./proto/*.proto
docker compose --env-file .env up

#******************************************************************************
 # Audi Autonomous Driving Cup 2018
 # Team frAIsers
 # AUTHOR: Fabien Jenne
 # 
 # Shell script to compile .thrift files for C++ & Python
 # 
#******************************************************************************

if [ "$1" = "" ]; then
    echo "Usage: ./makeThrift.sh <filename.thrift> <output directory>"
    exit
elif [ "$2" = "" ]; then
    echo "Usage: ./makeThrift.sh <filename.thrift> <output directory>"
    exit
fi

THRIFT_FILE_NAME=$(basename "$1" )
TARGET_DIR=$2
CURRENT_DIR=$PWD
BUILD_DIR="build"

if [ -d "$TARGET_DIR/gen-py" ]; then
    #rm -rf "$TARGET_DIR/gen-py"
    echo "overwriting files in $TARGET_DIR/gen-py"
fi

if [ -d "$TARGET_DIR/gen-cpp" ]; then
    #rm -rf "$TARGET_DIR/gen-cpp"
    echo "overwriting files in $TARGET_DIR/gen-cpp"
fi

if [ ! -d "./$BUILD_DIR" ]; then
    mkdir "./$BUILD_DIR"
fi
cp $1 "$BUILD_DIR/$THRIFT_FILE_NAME"
cd $BUILD_DIR
thrift -gen py "$THRIFT_FILE_NAME"
thrift -gen cpp "$THRIFT_FILE_NAME"
#rm -rf "$THRIFT_FILE_NAME"
cd $CURRENT_DIR
cp -ar "./$BUILD_DIR/." "$TARGET_DIR/" 
rm -r "./$BUILD_DIR"
#cp -r ./gen-py/* .

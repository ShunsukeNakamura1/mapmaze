//地図から迷路を作ります
//クラスを使用
//うまくいかないかも

#include "DxLib.h"
#define MAPSIZE 500 //最大マップサイズ

class Map{
	enum{
		ROAD = 0, //道
		WALL = 1, //壁
	};

private:
	char map[MAPSIZE][MAPSIZE];
	int height, width;
	unsigned int white,  black;

public:
	/*コンストラクタ
	読み込み画像の名前を引数とする*/
	Map(char* name){
		int handle, r, g, b, a;
		handle = LoadSoftImage(name);
		GetSoftImageSize( handle, &width, &height );
		for(int i = 0; i < height && i < MAPSIZE; i ++ ){
			for(int j = 0; j < width && j < MAPSIZE; j ++ ){
				// １ドットの色を取得
				GetPixelSoftImage( handle, j, i, &r, &g, &b, &a ) ;
				if(r >= 230 && g >= 230 && b >= 230){
					map[j][i] = ROAD;
				}else{
					map[j][i] = WALL;
				}
			}
		}
		white = GetColor(255, 255, 255);
		black = GetColor(  0,   0,   0);
		DeleteSoftImage( handle );
	}

	//地図の描画
	void drawMap(int x_gap, int y_gap){
		for(int i = 0; i < height && i < MAPSIZE; i ++ ){
			for(int j = 0; j < width && j < MAPSIZE; j ++ ){
	            if(map[j][i] == ROAD){
					DrawBox(j, i, j + 1, i + 1, white, TRUE);
				}else if(map[j][i] == WALL){
					
				}

			}
		}
	}
};

int WINAPI WinMain( HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow )
{
	// ウインドウモードに変更
    ChangeWindowMode( TRUE );

    // ＤＸライブラリの初期化
    if( DxLib_Init() < 0 ){
		return -1;
	}
    // マップクラス初期化
    Map map("map1.PNG");

    //描画
	map.drawMap(0, 0);

    // キー入力待ち
    WaitKey();

    // ＤＸライブラリの後始末
    DxLib_End();

    // ソフトの終了
    return 0;
}
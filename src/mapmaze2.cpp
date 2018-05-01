//地図から迷路を作りまーす
//スタート・ゴール未実装

#include "DxLib.h"
#include <stack>
#include <vector>

#define FILENAME "small1.PNG" //読み込むファイルの名前
//#define FILENAME "a.png" //読み込むファイルの名前
//#define FILENAME "map4.PNG" //読み込むファイルの名前
//#define FILENAME "b.png" //読み込むファイルの名前 簡単なやつ
#define MAPSIZE 1500//最大マップサイズ
#define WALL 0      //壁
#define ROAD 1      //道
#define VERTEX 2    //頂点
#define WINDOWX 1000//ウィンドウサイズ
#define WINDOWY 800
#define SMALL 500   //ラベル付けされた領域の面積がSMALL以下だと小領域とする
#define CUTCOUNT 200//道の接続を切る回数

//こーぞーたい
struct tMouse{ //マウスイベントのための構造体
	int x;		//x方向のずれ
	int y;		//y方向のずれ
	int size;	//マップのサイズ
};
struct XY{
	int x;
	int y;
};
struct tVer{  //頂点
	tVer(){}
	tVer(int x_, int y_){
		x = x_;
		y = y_;
	}
	tVer(int number, int x_, int y_){
		num = number;
		x = x_;
		y = y_;
	}
	int num;  //頂点番号
	int x;    //x座標
	int y;    //y座標
};
struct tEdge{  //辺
	tEdge(){}
	tEdge(tVer s, tVer d,int cost_, int angle_){
		src = s;
		dst = d;
		cost = cost_;
		angle = angle_;
	}
	tVer src;  //頂点1
	tVer dst;  //頂点2
	int cost;
	//辺探索用(頂点srcから頂点dstがどの方向にあるか)
	//  2
	//1 ! 3
	//  0
	int angle; 
	
};
/*  ユニオンファインド 参考:http://dai1741.github.io/maximum-algo-2012/docs/minimum-spanning-tree/  */
struct UnionFind{
	// par[i]：データiが属する木の親の番号。i == par[i]のとき、データiは木の根ノードである
	std::vector<int> par;
	// sizes[i]：根ノードiの木に含まれるデータの数。iが根ノードでない場合は無意味な値となる
	std::vector<int> sizes;

	UnionFind(int n) : par(n), sizes(n, 1) {
		// 最初は全てのデータiがグループiに存在するものとして初期化
		for (int i=0;i < n;i++){
			par[i] = i;
		}
	}

	// データxが属する木の根を得る
	int find(int x) {
		if (x == par[x]) return x;
		return par[x] = find(par[x]);  // 根を張り替えながら再帰的に根ノードを探す
	}

	// 2つのデータx, yが属する木をマージする
	void unite(int x, int y) {
		// データの根ノードを得る
		x = find(x);
		y = find(y);

		// 既に同じ木に属しているならマージしない
		if(x == y){
			return;
		}

		// xの木がyの木より大きくなるようにする
		if (sizes[x] < sizes[y]) std::swap(x, y);

		// xがyの親になるように連結する
		par[y] = x;
		sizes[x] += sizes[y];
		// sizes[y] = 0;  // sizes[y]は無意味な値となるので0を入れておいてもよい
	}

	// 2つのデータx, yが属する木が同じならtrueを返す
	bool same(int x, int y) {
		return find(x) == find(y);
	}

	// データxが含まれる木の大きさを返す
	int size(int x) {
		return sizes[find(x)];
	}
};
//int height, width;
//関数共
/*  画像nameを読み込んでmap[][]に二値化して保存  */
void loadMap(char *name, int **map, int *height, int *width);
/*  地図の描画  */
void drawMap(int **map, int height, int width, tMouse d);
/*  ラベル番号を指定した地図の描画  */
void drawMap_label(int **map, int height, int width, tMouse d, int label);
/*  頂点番号を指定した地図の描画  */
void drawMap_ver(int **map, int height, int width, tMouse d, tVer ver);
/*  辺番号を指定した地図の描画  */
void drawMap_edge(int **map, int height, int width, tMouse d, tEdge edge);
/*  画面の描画・保存  */
void saveScreen(int **map, int height, int width, tMouse d, char *filename);
/*  マウスのイベント処理  */
tMouse mouseEvent(tMouse d);
/*  細線化  */
void thinning(int **map, int height, int width);
int N(int *v);
int S(int *v);
/*  ラベリング 戻り値はラベル数  */
int labeling(int **map, int height, int width, int *label_area);
void search(int **map,int map_label[][MAPSIZE], int w, int h, int x, int y, int label, int *label_area);
/*  小領域をつなげる  */
void connectSmallArea(int **map, int height, int width, int *label_area);
/*  小領域の削除  */
void deleteSmallArea(int **map, int height, int width, int *label_area);
bool search2(int **map, int height, int width, int startx, int starty, int goalx, int goaly);
/*  迷路作製1(ラベリングを使用)  */
void makeMaze1(int **map, int height, int width, int cut_count, int *label_area);
/*  迷路作製2(探索を使用)  */
void makeMaze2(int **map, int height, int width, int cut_count);
/*  迷路作製3(グラフを使用)  */
void makeMaze3(int **map, int height, int width, int vernum, std::vector<tEdge> &edge, tMouse d);
/*  頂点(交差点)登録 戻り値は頂点数  */
int entryVer(int **map, int height, int width, std::vector<tVer> &ver);
int countAround(int **map, int x, int y);
/*  辺登録  */
int entryEdge(int **map, int height, int width, std::vector<tEdge> &edge, std::vector<tVer> &ver);
/*  辺の削除  */
bool deleteEdge(int **map, int height, int width, tEdge edge);
/*  tEdgeのvector配列のシャッフル  */
void shuffleEdge(std::vector<tEdge> &edge);
/*  tEdge.src.num・tEdge.dst.num と tVer.numとの対応付け  */
void associateEdgeNumber(std::vector<tVer> &ver, std::vector<tEdge> &edge);

int WINAPI WinMain( HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow )
{
	//各種設定
	//SetGraphMode(WINDOWX, WINDOWY, 32);
    ChangeWindowMode( TRUE );
	SetAlwaysRunFlag(TRUE);

    // ＤＸライブラリの初期化
    if( DxLib_Init() < 0 ){
		return -1;
	}

	//変数共
	static int map[MAPSIZE][MAPSIZE];//地図のほぞん先
	int *map_address[MAPSIZE];		 //その名の通り
	int height, width;				 //地図の幅
	int save_height, save_width;	 //保存用の地図の幅(heightやwidthとMAPSIZEのどちらかの小さい方)
	tMouse d = {0, 0, 1};            //mapの中身を表示する際の情報
	tMouse d_old = d;
	int dispmode=3;                  //描画モード

	int labelnum;                    //総ラベル数
	int watch_label = 1;             //見たいラベル番号
	int label_area[10000]={0};       //各ラベルの面積

	int vernum;                      //頂点の数
	std::vector<tVer> ver;           //頂点登録用
	int watch_ver = 0;               //見たい頂点番号

	int edgenum;                     //辺の数
	std::vector<tEdge> edge;         //辺登録用
	int watch_edge = 0;              //見たい辺番号

	for(int i=0;i<MAPSIZE;i++){
		map_address[i] = map[i];
	}

	//****************  前処理ここから   ****************//

    //マップ読み込み(二値化)
    loadMap(FILENAME, map_address, &height, &width);
	SetGraphMode(width, height, 32);
	//保存領域の設定
	//800 * 800位が限界っぽい
	if(height < MAPSIZE){
		save_height = height;
	}else{
		save_height = MAPSIZE;
	}
	if(width < MAPSIZE){
		save_width = width;
	}else{
		save_width = MAPSIZE;
	}
	//800 * 800位が限界っぽいので・・・
	if(save_height > 800) save_height = 800;
	if(save_width > 800) save_width = 800;

	//描画して保存
	saveScreen(map_address, save_height, save_width, d, "二値化.bmp");

	//細線化
	thinning(map_address, height, width);

    //描画して保存
	saveScreen(map_address, save_height, save_width, d, "細線化.bmp");

	//ラベリング
	labelnum = labeling(map_address, height, width, label_area);

	//小領域をつなげYo!
	connectSmallArea(map_address, height, width, label_area);
	for(int i = 0; i < height && i < MAPSIZE; i ++ ){
		for(int j = 0; j < width && j < MAPSIZE; j ++ ){
			if(map[j][i] > ROAD){
				map[j][i] = ROAD;
			}
		}
	}
	
	//描画して保存
	saveScreen(map_address, save_height, save_width, d, "小領域接続.bmp");

	thinning(map_address, height, width);
	labelnum = labeling(map_address, height, width, label_area);
	
	//小領域を削除しYo!
	deleteSmallArea(map_address, height, width, label_area);
	for(int i = 0; i < height && i < MAPSIZE; i ++ ){
		for(int j = 0; j < width && j < MAPSIZE; j ++ ){
			if(map[j][i] > ROAD){
				map[j][i] = ROAD;
			}
		}
	}
	
	//描画して保存
	saveScreen(map_address, save_height, save_width, d, "小領域削除.bmp");

	labelnum = labeling(map_address, height, width, label_area);
	
	if(labelnum > 1){
		printfDx("Error:ラベルが2以上あるよ");
		ScreenFlip();
		WaitKey();
		exit(1);
	}

	//頂点の登録
	vernum = entryVer(map_address, height,width, ver);
	//辺の登録
	edgenum = entryEdge(map_address, height, width, edge, ver);
	//edgeに登録した辺のsrc・dstの番号をverに登録してある番号にあわせる
	associateEdgeNumber(ver, edge);
	shuffleEdge(edge);
	shuffleEdge(edge);
	mouseEvent(d);
	//****************  前処理ここまで   ****************//

	//****************  迷路作製ここから  ****************//
	//makeMaze1(map_address, height, width, CUTCOUNT, label_area);
	//makeMaze2(map_address, height, width, CUTCOUNT);
	makeMaze3(map_address, height, width, vernum, edge, d);
	//****************  迷路作製ここまで  ****************//

	//唐突にマウスイベントを実行
	mouseEvent(d);
	//頂点の登録
	vernum = entryVer(map_address, height,width, ver);
	//辺の登録
	edgenum = entryEdge(map_address, height, width, edge, ver);

	//**************************  ここから表示  **************************//
	bool isdraw = true;
	bool ishide = false;//文字を隠すか
	while (ProcessMessage() == 0 && CheckHitKey(KEY_INPUT_ESCAPE) == 0){
		clsDx();
		d = mouseEvent(d);   //マウスのイベント処理
		if(d.x != d_old.x || d.y != d_old.y || d.size != d_old.size || isdraw){
			clsDx();
			ClearDrawScreen();
			switch(dispmode){
			case 0://マップの描画
				drawMap(map_address, height, width, d);
				break;
			case 1://ラベルを指定したマップの描画
				drawMap_label(map_address, height, width, d, watch_label);
				break;
			case 2://頂点を指定したマップの描画
				drawMap_ver(map_address, height, width, d, ver[watch_ver]);
				break;
			case 3://辺番号を指定したマップの描画
				drawMap_edge(map_address, height, width, d, edge[watch_edge]);
				break;
			}
			if(!ishide){
				switch(dispmode){
				case 0://マップの描画
					printfDx("edgenum:%d\n",edgenum);
					break;
				case 1://ラベルを指定したマップの描画
					printfDx("label:%d  area:%d labelnum:%d\n",watch_label,label_area[watch_label],labelnum);
					break;
				case 2://頂点を指定したマップの描画
					printfDx("ver:%d x:%d y:%d  vernum:%d\n",ver[watch_ver].num, ver[watch_ver].x, ver[watch_ver].y,vernum);
					break;
				case 3://辺番号を指定したマップの描画
					printfDx("edge:%d cost:%d   edgenum:%d\n", watch_edge, edge[watch_edge].cost, edgenum);
					printfDx("src_x:%d src_y:%d\n", edge[watch_edge].src.x, edge[watch_edge].src.y);
					printfDx("dst_x:%d dst_y:%d\n", edge[watch_edge].dst.x, edge[watch_edge].dst.y);
					break;
				}
				printfDx(" a : 描画モード変更\n");
				printfDx(" b : 設定変更\n");
				printfDx(" c : 文字を隠す/表示する\n");
				printfDx(" d : 辺の削除\n");
				printfDx(" e : 画面の保存\n");
			}
			ScreenFlip();
			isdraw = false;
		}
		//描画モードの変更
		if(CheckHitKey( KEY_INPUT_A ) == 1){
			WaitTimer(100);
			clsDx();
			ClearDrawScreen();
			ScreenFlip();
			printfDx("描画モードの変更\n");
			printfDx(dispmode==0 ? "○" : "  ");
			printfDx(" 0 : マップの描画\n");
			printfDx(dispmode==1 ? "○" : "  ");
			printfDx(" 1 : ラベルを指定したマップの描画\n");
			printfDx(dispmode==2 ? "○" : "  ");
			printfDx(" 2 : 頂点を指定したマップの描画\n");
			printfDx(dispmode==3 ? "○" : "  ");
			printfDx(" 3 : 辺を指定したマップの描画\n");
			dispmode = KeyInputNumber( 0 , 96 , 3 , 0, FALSE);
			isdraw = true;
		}
		//各描画の設定変更
		else if(CheckHitKey( KEY_INPUT_B ) == 1){
			WaitTimer(100);
			clsDx();
			ClearDrawScreen();
			ScreenFlip();
			switch(dispmode){
			case 0:
				break;
			case 1:
				printfDx("ラベル番号を入力>");
				watch_label = KeyInputNumber( 0 , 32 , labelnum , 1, FALSE);
				break;
			case 2:
				printfDx("頂点番号を入力>");
				watch_ver = KeyInputNumber( 0 , 16 , vernum-1 , 0, FALSE);
				break;
			case 3:
				printfDx("辺番号を入力>");
				watch_edge = KeyInputNumber( 0 , 16 , edgenum-1 , 0, FALSE);
				break;
			}
			clsDx();
			ClearDrawScreen();
			isdraw = true;
		}
		//文字を隠しますか?
		else if(CheckHitKey( KEY_INPUT_C ) == 1){
			WaitTimer(100);
			ishide = !ishide;
			isdraw = true;
		}
		//辺の削除
		else if(CheckHitKey( KEY_INPUT_D ) == 1){
			WaitTimer(100);
			clsDx();
			ClearDrawScreen();
			printfDx("削除したい辺を入力してください\n");
			int delete_edge = KeyInputNumber( 0 , 32 , edgenum-1 , 0, FALSE);
			deleteEdge(map_address, height, width, edge[delete_edge]);
			isdraw =true;
		}
		//画面の保存
		else if(CheckHitKey( KEY_INPUT_E ) == 1){
			WaitTimer(100);
			saveScreen(map_address, save_height, save_width, d, "迷路.bmp");
			isdraw =true;
		}
		//注目する辺を1増やす
		else if(CheckHitKey( KEY_INPUT_SPACE ) == 1){
			WaitTimer(100);
			watch_edge++;
			isdraw =true;
		}
		d_old = d;
	}
	//**************************  ここまで表示  **************************//

    // ＤＸライブラリの後始末
    DxLib_End();

    // ソフトの終了
    return 0;
}


//関数共
//画像nameを読み込んでmap[][]に二値化して保存
void loadMap(char *name, int **map, int *height, int *width){
	int handle, r, g, b, a;
	handle = LoadSoftImage(name);
	GetSoftImageSize( handle, width, height );
	for(int i = 0; i < *height && i < MAPSIZE; i ++ ){
		for(int j = 0; j < *width && j < MAPSIZE; j ++ ){
			// １ドットの色を取得
			GetPixelSoftImage( handle, j, i, &r, &g, &b, &a ) ;
			//if(r >= 230 && g >= 230 && b >= 230){
			if(r >= 85 && g >= 85 && b >= 85){
				map[j][i] = ROAD;
			}else{
				map[j][i] = WALL;
			}
		}
	}
	DeleteSoftImage( handle );
}

//細線化
/*参考:http://www.cellstat.net/thinning/
細線化は2つの処理X,Yを連続で適用する
注目画素とその周りの値 v[9] を設定
		v[8] v[1] v[2]
		v[7] v[0] v[3]
		v[6] v[5] v[4]

さらに条件判定のための関数N(v)とS(v)を定義
N(v):v[1]～v[8]のうち1(道)であるものの数
S(v):v[1] > v[2] > v[3] > v[4] > … > v[7] > v[8] > v[1] と一周するうちで隣り合う2つが0と1で異なるものの数

処理X
画像の各画素を処理するためにx,yについての2重のforループを回し、元々の画素の値が1(道)だったものに対して、
以下の条件を同時に満たす場合には値を0(壁)に更新
条件1: 2 <= N(v) <= 6
条件2: S(v) == 1
条件3: v[1] * v[3] * v[5] == 0
条件4: v[3] * v[5] * v[7] == 0

処理Y
画像の各画素を処理するためにx,yについての2重のforループを回し、元々の画素の値が1(道)だったものに対して、
以下の条件を同時に満たす場合には値を0(壁)に更新
条件1: 2 <= N(v) <= 6
条件2: S(v) == 1
条件3: v[1] * v[3] * v[7] == 0
条件4: v[1] * v[5] * v[7] == 0
*/
void thinning(int **map, int height, int width){
	bool complete = false; //更新が行われたらfalseになるtrueになれば細線化終了
	int count=0;
	while(!complete){
		complete = true;
		int v[9];

		for(int i = 1; i < height-1 && i < MAPSIZE-1; i ++ ){
			for(int j = 1; j < width-1 && j < MAPSIZE-1; j ++ ){
				//v[]の設定
				v[8]=map[j-1][i-1]; v[1]=map[ j ][i-1]; v[2]=map[j+1][i-1];
				v[7]=map[j-1][ i ]; v[0]=map[ j ][ i ]; v[3]=map[j+1][ i ];
				v[6]=map[j-1][i+1]; v[5]=map[ j ][i+1]; v[4]=map[j+1][i+1];
				//処理X
				if(map[j][i] == ROAD){
					if(2 <= N(v) && N(v) <= 6 && S(v) == 1 && v[1] * v[3] * v[5] == 0 && v[3] * v[5] * v[7] == 0){
						map[j][i] = WALL;
						complete = false;
					}
				}
				//処理Y
				if(map[j][i] == ROAD){
					if(2 <= N(v) && N(v) <= 6 && S(v) == 1 && v[1] * v[3] * v[7] == 0 && v[1] * v[5] * v[7] == 0){
						map[j][i] = WALL;
						complete = false;
					}
				}
			}
		}
	}
}
int N(int *v){
	int count = 0;
	for(int i=1;i<9;i++){
		if(v[i] == ROAD){
			count++;
		}
	}
	return count;
}
int S(int *v){
	int count = 0;
	int old = v[8];
	for(int i=1;i<9;i++){
		if(old == 0 && v[i] == 1){
			count++;
		}
		old = v[i];
	}
	return count;
}

/*再帰記念品(贈呈:宮岡)

int getFinalDst(int i,int* dst){
	if(i == dst[i])
		return i;
	dst[i] = getFinalDst(dst[i], dst);
	return dst[i];
}


int labelClose(int* dst){
	int cnv[15000] = {0};
	int label_num = 0;
	for(int i=1;i<15000;i++){
		if(cnv[ dst[i] ] == 0){
			label_num++;
			cnv[ dst[i] ] = label_num;
		}
	}
	for(int i=1;i<15000;i++){
		dst[i] = cnv[ dst[i] ];
	}
	return label_num;
}


//ラベリング 戻り値はラベル数
//参考：http://imagingsolution.blog107.fc2.com/blog-entry-193.html

int labeling(int **map, int height, int width){
	static int map_label[MAPSIZE][MAPSIZE]={0}; //ラベリング用のマップ
	int labelnum=0;	//ラベル数 これを返す
	int label_max=0;
	//int src[15000]; //ルックアップテーブル
	int dst[15000];

	for(int i=0;i<15000;i++){//ルックアップテーブルの初期化
		//src[i] = i;
		dst[i] = i;
	}

	//これよりラベリングをはじめます
	//ラスタスキャン
	//先に一番左上を見ておく
	if(map[0][0] == ROAD){
		map_label[0][0] = ++labelnum;
	}

	for(int i = 0; i < height && i < MAPSIZE; i ++ ){
		for(int j = 0; j < width && j < MAPSIZE; j ++ ){
			if(i == 0 && j == 0) continue;//map[0][0]はとばす

			if(j==0){//一番上の行は上を参照しない
				if(map[j][i] == ROAD){//注目画素が道なら左を見る
					if(map_label[j][i-1] != WALL){//左が道ならそのラベルを注目画素にもつける
						map_label[j][i] = map_label[j][i-1];
					}else{//違えば最後にふったラベル番号のひとつ大きな番号をふる
						map_label[j][i] = ++labelnum;
					}
				}
			}else if(i==0){//一番左の行は左を参照しない
				if(map[j][i] == ROAD){//注目画素が道なら上を見る
					if(map[j-1][i] != WALL){//上が道ならそのラベルを注目画素にもつける
						map_label[j][i] = map_label[j-1][i];
					}else{//違えば最後にふったラベル番号のひとつ大きな番号をふる
						map_label[j][i] = ++labelnum;
					}
				}
			}else{//それ以外のところは左と上を参照する
				if(map[j][i] == ROAD){
					if(mini(map_label[j][i-1], map_label[j-1][i]) != -1){//両方とも0でなければ
						map_label[j][i] = mini(map_label[j][i-1], map_label[j-1][i]);//注目画素のラベル番号を小さいほうのラベル番号にあわせる
						int new_dst = mini(map_label[j][i-1], map_label[j-1][i]);
						while(new_dst != )
						dst[ dst[max(map_label[j][i-1], map_label[j-1][i])] ] = new_dst;//前に入っていたやつも書き換えないと・・・
						dst[max(map_label[j][i-1], map_label[j-1][i])] = new_dst;//ルックアップテーブルに書き込む

					}else{//両方0ならば新たなラベル番号をつける
						map_label[j][i] = ++labelnum;
					}
				}
			}
		}
	}
	//数字をつめるとこby宮岡
	for(int i=1;i<15000;i++){
		getFinalDst(i, dst);
	}
	int label_all = labelClose(dst);
	//ここまでby宮岡
	clsDx();
	ClearDrawScreen();
	printfDx("labeling completed!!\nlabel:%d\n",labelnum);
	ScreenFlip();

	//ルックアップテーブルを使って対応させる
	for(int i = 0; i < height && i < MAPSIZE; i ++ ){
		for(int j = 0; j < width && j < MAPSIZE; j ++ ){
			if(map[j][i] != 0){
				map[j][i] = dst[ map_label[j][i] ];
				label_max = max(label_max, map[j][i]);
			}
		}
	}

	return label_max;
}
//0以外の小さいほうを返す 両方0なら-1を返す
int mini(int a,int b){
	if(a == 0 && b == 0){
		return -1;
	}else{
		if(a == 0){
			return b;
		}else if(b == 0){
			return a;
		}else if(a < b){
			return a;
		}else{
			return b;
		}
	}
}
ここまで再帰記念品
*/



int labeling(int **map, int height, int width, int *label_area){
	static int map_label[MAPSIZE][MAPSIZE]={0}; //ラベリング用のマップ
	int labelnum=0;	//ラベル数 これを返す

	for(int i = 0; i < height && i < MAPSIZE; i ++ ){
		for(int j = 0; j < width && j < MAPSIZE; j ++ ){
			map_label[j][i] = 0;
		}
	}
	for(int i = 0; i < height && i < MAPSIZE; i ++ ){
		for(int j = 0; j < width && j < MAPSIZE; j ++ ){
			if(map[j][i] == ROAD && map_label[j][i] == 0){
				search(map, map_label, width, height, j, i, ++labelnum, label_area);
			}
		}
	}
	for(int i = 0; i < height && i < MAPSIZE; i ++ ){
		for(int j = 0; j < width && j < MAPSIZE; j ++ ){
			map[j][i] = map_label[j][i];
		}
	}
	return labelnum;
}

void search(int **map,int map_label[][MAPSIZE], int w, int h, int x, int y, int label, int *label_area){
	//ラベリングの探索 藤原さん作
	std::stack<std::pair<int, int>> p;
	p.push(std::make_pair(x, y));
	int count = 0;
	while(! p.empty()) {
		std::pair<int, int> pos = p.top();
		p.pop();
		int px = pos.first;
		int py = pos.second;
		map_label[px][py] = label;
		label_area[label]++;
		if (px - 1 >= 0 && map_label[px-1][py] == 0 && map[px-1][py] == ROAD) p.push(std::make_pair(px - 1, py));
		if (px + 1 < w && map_label[px+1][py] == 0 && map[px+1][py] == ROAD) p.push(std::make_pair(px + 1, py));
		if (py - 1 >= 0 && map_label[px][py-1] == 0 && map[px][py-1] == ROAD) p.push(std::make_pair(px, py - 1));
		if (py + 1 < h && map_label[px][py+1] == 0 && map[px][py+1] == ROAD) p.push(std::make_pair(px, py + 1));
	}
}
/*  小領域をつなげる  
ラベル付けされた領域の面積がSMALL以下の小領域に対して
斜め方向の連結を見て、そこが白画素(道)かつ自身と違うラベル番号なら
そこをつなげる
               *                 *                 *
2 0 0   2 0 0  *  0 0 2   0 0 2  *  0 0 0   0 0 0  *  0 0 0   0 0 0
0 1 0 > 1 1 0  *  0 1 0 > 0 1 1  *  0 1 0 > 1 1 0  *  0 1 0 > 0 1 1
0 0 0   0 0 0  *  0 0 0   0 0 0  *  2 0 0   2 0 0  *  0 0 2   0 0 2
               *                 *                 *
*/
void connectSmallArea(int **map, int height, int width, int *label_area){
	for(int i = 0; i < height && i < MAPSIZE; i ++ ){
		for(int j = 0; j < width && j < MAPSIZE; j ++ ){
			if(map[j][i] != 0 && label_area[ map[j][i] ] <= SMALL){
				if(i != 0 && j != 0){
					if(map[j-1][i-1] >= 1 && map[j][i] != map[j-1][i-1]) map[j-1][i] = ROAD;
				}
				if(i != 0){
					if(map[j+1][i-1] >= 1 && map[j][i] != map[j+1][i-1]) map[j+1][i] = ROAD;
				}
				if(j != 0){
					if(map[j-1][i+1] >= 1 && map[j][i] != map[j-1][i+1]) map[j-1][i] = ROAD;
				}
				if(map[j+1][i+1] >= 1 && map[j][i] != map[j+1][i+1]) map[j+1][i] = ROAD;
			}
		}
	}
}

/*  小領域の削除  */
void deleteSmallArea(int **map, int height, int width, int *label_area){
	for(int i = 0; i < height && i < MAPSIZE; i ++ ){
		for(int j = 0; j < width && j < MAPSIZE; j ++ ){
			if(label_area[ map[j][i] ] <= SMALL){
				map[j][i] = 0;
			}
		}	
	}
}

/*  迷路作製1(ラベリングを使用)  */
void makeMaze1(int **map, int height, int width, int cut_count, int *label_area){
	//XY a[10];
	int count = 0;
	while(cut_count != 0){
		//ランダムに選んだ点を黒画素に変える
		/*
		for(int i=0;i<10;i++){
			int x = GetRand(min(width,MAPSIZE));
			int y = GetRand(min(height,MAPSIZE));
			if(map[x][y] == WALL){
				i--;
				continue;
			}
			map[x][y] = WALL;
			a[i].x = x; a[i].y = y;
		}
		*/
		///*
		for(int i = 0; i < height && i < MAPSIZE; i ++ ){
			for(int j = 0; j < width && j < MAPSIZE; j ++ ){
				if(map[j][i] > ROAD){
					map[j][i] = ROAD;
				}
			}
		}
		int x = GetRand(min(width,MAPSIZE))+1;
		int y = GetRand(min(height,MAPSIZE))+1;
		if(map[x][y] == WALL){
			continue;
		}
		map[x][y] = WALL;
		//*/
		//ラベリングを使った連結性確認法
		//ClearDrawScreen();
		//printfDx("making… \n");
		//ScreenFlip();
		if(labeling(map, height, width, label_area) == 1){
			cut_count--;
			//clsDx();
			ClearDrawScreen();
			//printfDx("making… cut_count:%d  x:%d  y:%d\n",cut_count,x,y);
			printfDx("making… cut_count:%d\n",cut_count);
			ScreenFlip();
		}else{
			//ClearDrawScreen();
			//printfDx("making… false\n");
			//ScreenFlip();
			//for(int i=0;i<10;i++){
			//	map[a[i].x][a[i].y] = ROAD;
			//}
			map[x][y]=ROAD;
		}
		//cut_count--;
		ClearDrawScreen();
		//printfDx("making… cut_count:%d\n",cut_count);
		ScreenFlip();
	}
}

/*  迷路作製2(探索を使用)  */
void makeMaze2(int **map, int height, int width, int cut_count){
	while(cut_count != 0){
		//ランダムに選んだ点を黒画素に変える
		
		int x = GetRand(min(width,MAPSIZE))+1;
		int y = GetRand(min(height,MAPSIZE))+1;
		if(map[x][y] == WALL)continue;
		map[x][y] = WALL;
		//**********    探索を使った連結性確認法    **********//
		//  ☆
		//☆■□
		//  □
		if(map[x-1][y] == ROAD && map[x][y-1] == ROAD){
			if(search2(map,height, width, x-1, y, x, y-1)){}
			else{
				map[x][y]=ROAD;
				continue;
			}
		}
		//  □
		//☆■☆
		//  □
		if(map[x-1][y] == ROAD && map[x+1][y] == ROAD){
			if(search2(map,height, width, x-1, y, x+1, y)){}
			else{
				map[x][y]=ROAD;
				continue;
			}
		}
		//  □
		//☆■□
		//  ☆
		if(map[x-1][y] == ROAD && map[x][y+1] == ROAD){
			if(search2(map,height, width, x-1, y, x, y+1)){}
			else{
				map[x][y]=ROAD;
				continue;
			}
		}
		//  ☆
		//□■☆
		//  □
		if(map[x][y-1] == ROAD && map[x+1][y] == ROAD){
			if(search2(map,height, width, x, y-1, x+1, y)){}
			else{
				map[x][y]=ROAD;
				continue;
			}
		}
		//  ☆
		//□■□
		//  ☆
		if(map[x][y-1] == ROAD && map[x][y+1] == ROAD){
			if(search2(map,height, width, x, y-1, x, y+1)){}
			else{
				map[x][y]=ROAD;
				continue;
			}
		}
		//  □
		//□■☆
		//  ☆
		if(map[x+1][y] == ROAD && map[x][y+1] == ROAD){
			if(search2(map,height, width, x+1, y, x, y+1)){}
			else{
				map[x][y]=ROAD;
				continue;
			}
		}
		if(CheckHitKey( KEY_INPUT_S ) == 1){
			break;
		}
		//**********    探索を使った連結性確認法ここまで    **********//
		cut_count--;
		ClearDrawScreen();
		printfDx("making… cut_count:%d\n",cut_count);
		ScreenFlip();
	}
}

/*  迷路作製3(グラフを使用)  */
void makeMaze3(int **map, int height, int width, int vernum, std::vector<tEdge> &edge, tMouse d){
	UnionFind uf(vernum);
	clsDx();
	ClearDrawScreen();
	printfDx("Map making start...\n");
	ScreenFlip();
	int count=0;
	for(int i=0;i<edge.size();i++){
		//srcとdstが同じグループに属していなければその辺は削除せずにsrcとdstを同じグループにする
		if(!uf.same(edge[i].src.num, edge[i].dst.num)){
			uf.unite(edge[i].src.num, edge[i].dst.num);
		}
		//srcとdstが同じグループに属していたらその辺を削除
		else{
			count++;
			deleteEdge(map, height, width, edge[i]);
		}
	}
	clsDx();
	ClearDrawScreen();
	printfDx("Map making end!!!!\ncount : %d\n",count);
	ScreenFlip();
	WaitTimer(1000);
}

/*  連結しているかの確認  */
bool search2(int **map, int height, int width, int startx, int starty, int goalx, int goaly){
	std::vector<std::vector<char>> map_flag;
	map_flag.resize(MAPSIZE);
	for( int i=0; i<MAPSIZE; i++ ){
		map_flag[i].resize(MAPSIZE);
	}
	std::stack<std::pair<int, int>> p;
	p.push(std::make_pair(startx, starty));
	while(! p.empty()) {
		std::pair<int, int> pos = p.top();
		p.pop();
		int px = pos.first;
		int py = pos.second;
		if(px == goalx && py == goaly){
			return true;
		}
		map_flag[px][py] = 1;
		if (px - 1 >= 0     && map_flag[px-1][py] == 0 && map[px-1][py] == ROAD) p.push(std::make_pair(px - 1, py));
		if (px + 1 < width  && map_flag[px+1][py] == 0 && map[px+1][py] == ROAD) p.push(std::make_pair(px + 1, py));
		if (py - 1 >= 0     && map_flag[px][py-1] == 0 && map[px][py-1] == ROAD) p.push(std::make_pair(px, py - 1));
		if (py + 1 < height && map_flag[px][py+1] == 0 && map[px][py+1] == ROAD) p.push(std::make_pair(px, py + 1));
	}
	return false;
}

/*  地図の描画  */
void drawMap(int **map, int height, int width, tMouse d){
	static unsigned int white = GetColor(255, 255, 255);
	static unsigned int red = GetColor(255, 0, 0);
	static unsigned int blue = GetColor(0, 0, 255);
	int x, y;

	for(int i = 0; i < height && i < MAPSIZE; i ++ ){
		for(int j = 0; j < width && j < MAPSIZE; j ++ ){
			x = j * d.size + d.x;
			y = i * d.size + d.y;
			if(map[j][i] == ROAD){
				DrawBox(x, y, x + d.size, y + d.size, white, TRUE);
			}else if(map[j][i] == WALL){}
			else if(map[j][i] % 2 == 0){
				DrawBox(x, y, x + d.size, y + d.size, red, TRUE);
			}
			else if(map[j][i] % 2 == 1){
				DrawBox(x, y, x + d.size, y + d.size, blue, TRUE);
			}
		}
	}
}

/*  ラベル番号を指定した地図の描画  */
void drawMap_label(int **map, int height, int width, tMouse d, int label){
	static unsigned int white = GetColor(255, 255, 255);
	static unsigned int red = GetColor(255, 0, 0);
	static unsigned int blue = GetColor(0, 0, 255);
	int x, y;

	for(int i = 0; i < height && i < MAPSIZE; i ++ ){
		for(int j = 0; j < width && j < MAPSIZE; j ++ ){
			x = j * d.size + d.x;
			y = i * d.size + d.y;
			if(map[j][i] == WALL){}
			else if(map[j][i] == label){
				DrawBox(x, y, x + d.size, y + d.size, red, TRUE);
			}
			else{
				DrawBox(x, y, x + d.size, y + d.size, white, TRUE);
			}
		}
	}
	for(int i = 1; i < height && i < MAPSIZE; i ++ ){
		for(int j = 1; j < width && j < MAPSIZE; j ++ ){
			if(map[j][i] == ROAD && countAround(map, j, i) >= 3){
				x = j * d.size + d.x;
				y = i * d.size + d.y;
				DrawBox(x, y, x + d.size, y + d.size, white, TRUE);
			}
		}
	}
}

/*  頂点番号を指定した地図の描画  */
void drawMap_ver(int **map, int height, int width, tMouse d, tVer ver){
	static unsigned int white = GetColor(255, 255, 255);
	static unsigned int red = GetColor(255, 0, 0);
	static unsigned int blue = GetColor(0, 0, 255);
	int x, y;

	for(int i = 0; i < height && i < MAPSIZE; i ++ ){
		for(int j = 0; j < width && j < MAPSIZE; j ++ ){
			x = j * d.size + d.x;
			y = i * d.size + d.y;
			if(map[j][i] == WALL){}
			else if(map[j][i] == ROAD){
				DrawBox(x, y, x + d.size, y + d.size, white, TRUE);
			}
		}
	}
	x = ver.x * d.size + d.x;
	y = ver.y * d.size + d.y;
	DrawBox(x, y, x + d.size, y + d.size, red, TRUE);
}

/*  辺番号を指定した地図の描画  */
void drawMap_edge(int **map, int height, int width, tMouse d, tEdge edge){
	static unsigned int white = GetColor(255, 255, 255);
	static unsigned int red   = GetColor(255,   0,   0);
	static unsigned int green = GetColor(  0, 255,   0);
	static unsigned int blue  = GetColor(  0,   0, 255);
	int x,y;
	int srcx, srcy, dstx, dsty;
	//         下 左 上 右
	int dx[4]={-1, 0, 1, 0};
	int dy[4]={ 0,-1, 0, 1};
	//訪れたかどうかの確認用
	std::vector<std::vector<char>> map_flag;
	map_flag.resize(MAPSIZE);
	for( int i=0; i<MAPSIZE; i++ ){
		map_flag[i].resize(MAPSIZE);
	}

	for(int i = 0; i < height && i < MAPSIZE; i ++ ){
		for(int j = 0; j < width && j < MAPSIZE; j ++ ){
			x = j * d.size + d.x;
			y = i * d.size + d.y;
			if(map[j][i] == WALL){}
			else if(map[j][i] == ROAD){
				DrawBox(x, y, x + d.size, y + d.size, white, TRUE);
			}
		}
	}

	//src描画(赤)
	srcx = edge.src.x * d.size + d.x;
	srcy = edge.src.y * d.size + d.y;
	DrawBox(srcx, srcy, srcx + d.size, srcy + d.size, red, TRUE);
	//dst描画(青)
	dstx = edge.dst.x * d.size + d.x;
	dsty = edge.dst.y * d.size + d.y;
	DrawBox(dstx, dsty, dstx + d.size, dsty + d.size, blue, TRUE);

	//辺の描画
	std::stack<std::pair<int, int>> p;
	//辺のsrcのangle方向に描画しながらdstを探索
	p.push(std::make_pair(edge.src.x+dx[edge.angle], edge.src.y+dy[edge.angle]));
	map_flag[edge.src.x][edge.src.y] = 1;
	while(! p.empty()){
		std::pair<int, int> pos = p.top();
		p.pop();
		int px = pos.first;
		int py = pos.second;
		map_flag[px][py] = 1;
		//頂点dstを訪れるまでループ
		if(px == edge.dst.x && py == edge.dst.y){
			break;
		}else{
			x = px * d.size + d.x;
			y = py * d.size + d.y;
			DrawBox(x, y, x + d.size, y + d.size, green, TRUE);
			if (px - 1 >= 0     && map[px-1][py] != WALL && map_flag[px-1][py] == 0) p.push(std::make_pair(px - 1, py));
			if (px + 1 < width  && map[px+1][py] != WALL && map_flag[px+1][py] == 0) p.push(std::make_pair(px + 1, py));
			if (py - 1 >= 0     && map[px][py-1] != WALL && map_flag[px][py-1] == 0) p.push(std::make_pair(px, py - 1));
			if (py + 1 < height && map[px][py+1] != WALL && map_flag[px][py+1] == 0) p.push(std::make_pair(px, py + 1));
		}
	}
}

/*  画面の描画・保存  */
void saveScreen(int **map, int height, int width, tMouse d, char *filename){
	ClearDrawScreen();
	drawMap(map, height, width, d);
	ScreenFlip();
	SaveDrawScreen( 0, 0, width, height, filename);
}

/*  頂点(交差点)登録 戻り値は頂点数  */
int entryVer(int **map, int height, int width, std::vector<tVer> &ver){
	int ver_count = 0;
	for(int i = 1; i < height && i < MAPSIZE; i ++ ){
		for(int j = 1; j < width && j < MAPSIZE; j ++ ){
			if(map[j][i] == ROAD && countAround(map, j, i) >= 3){
				ver.push_back(tVer(ver_count++, j, i));
			}
		}
	}
	return ver_count;
}
int countAround(int **map, int x, int y){
	int count = 0;
	if(map[x-1][ y ] == ROAD) count++;if(map[x+1][ y ] == ROAD) count++;
	if(map[ x ][y-1] == ROAD) count++;
	if(map[ x ][y+1] == ROAD) count++;
	return count;
}

/*  辺の登録  */
int entryEdge(int **map, int height, int width, std::vector<tEdge> &edge, std::vector<tVer> &ver){
	int edgecount = 0;
	//         下 左 上 右
	int dx[4]={-1, 0, 1, 0};
	int dy[4]={ 0,-1, 0, 1};
	std::vector<std::vector<int>> mapcopy;
	mapcopy.resize(MAPSIZE);
	for( int i=0; i<MAPSIZE; i++ ){
		mapcopy[i].resize(MAPSIZE);
	}
	//mapのコピーをとる
	for(int i = 1; i < height && i < MAPSIZE; i ++ ){
		for(int j = 1; j < width && j < MAPSIZE; j ++ ){
			mapcopy[j][i] = map[j][i];
		}
	}
	//頂点はVERTEX(2)に変換
	for(int i=0;i<ver.size();i++){
		mapcopy[ver[i].x][ver[i].y] = VERTEX;
	}
	//頂点から探索
	for(int i=0;i<ver.size();i++){
		//四近傍探索
		for(int j=0;j<4;j++){//j=0(下),1(左),2(上),3(右)
			int cost=0;
			std::stack<std::pair<int, int>> p;
			if(ver[i].x+dx[j] < 0 || ver[i].y+dy[j] < 0 || ver[i].x+dx[j] >= width || ver[i].y+dy[j] >= height){
				continue;
			}
			if(mapcopy[ver[i].x+dx[j]][ver[i].y+dy[j]] == WALL){
				continue;
			}
			p.push(std::make_pair(ver[i].x+dx[j], ver[i].y+dy[j]));
			while(! p.empty()){
				std::pair<int, int> pos = p.top();
				p.pop();
				int px = pos.first;
				int py = pos.second;
				//もし訪問地点が頂点なら探索開始頂点と今いる頂点で辺を登録
				if(mapcopy[px][py] == VERTEX && (px != ver[i].x || py != ver[i].y)){
					tVer src(ver[i].x, ver[i].y);
					tVer dst(px, py);
					edge.push_back(tEdge(src, dst, cost, j));
					edgecount++;
					break;
				}
				cost++;
				mapcopy[px][py] = WALL;
				if (px - 1 >= 0     && mapcopy[px-1][py] != WALL && (px-1 != ver[i].x || py != ver[i].y)){
					p.push(std::make_pair(px - 1, py));
				}
				if (px + 1 < width  && mapcopy[px+1][py] != WALL && (px+1 != ver[i].x || py != ver[i].y)){
					p.push(std::make_pair(px + 1, py));
				}
				if (py - 1 >= 0     && mapcopy[px][py-1] != WALL && (px != ver[i].x || py-1 != ver[i].y)){
					p.push(std::make_pair(px, py - 1));
				}
				if (py + 1 < height && mapcopy[px][py+1] != WALL && (px != ver[i].x || py+1 != ver[i].y)){
					p.push(std::make_pair(px, py + 1));
				}
			}
		}
	}
	return edgecount;
}

/*  辺の削除  */
bool deleteEdge(int **map, int height, int width, tEdge edge){
	int dx[4]={-1, 0, 1, 0};
	int dy[4]={ 0,-1, 0, 1};
	//訪れたかどうかの確認用
	std::vector<std::vector<char>> map_flag;
	map_flag.resize(MAPSIZE);
	for( int i=0; i<MAPSIZE; i++ ){
		map_flag[i].resize(MAPSIZE);
	}
	//辺の探索
	std::stack<std::pair<int, int>> p;
	//辺のsrcのangle方向に削除しながらdstを探索
	p.push(std::make_pair(edge.src.x+dx[edge.angle], edge.src.y+dy[edge.angle]));
	map_flag[edge.src.x][edge.src.y] = 1;
	while(! p.empty()){
		std::pair<int, int> pos = p.top();
		p.pop();
		int px = pos.first;
		int py = pos.second;
		//頂点dstを訪れるまでループ
		if(px == edge.dst.x && py == edge.dst.y){
			return true;
		}else{
			map_flag[px][py] = 1;
			map[px][py] = WALL;
			if (px - 1 >= 0     && map[px-1][py] != WALL && map_flag[px-1][py] == 0) p.push(std::make_pair(px - 1, py));
			if (px + 1 < width  && map[px+1][py] != WALL && map_flag[px+1][py] == 0) p.push(std::make_pair(px + 1, py));
			if (py - 1 >= 0     && map[px][py-1] != WALL && map_flag[px][py-1] == 0) p.push(std::make_pair(px, py - 1));
			if (py + 1 < height && map[px][py+1] != WALL && map_flag[px][py+1] == 0) p.push(std::make_pair(px, py + 1));
		}
	}
	return false;
}

/*  tEdgeのvector配列のシャッフル  */
void shuffleEdge(std::vector<tEdge> &edge){
	//コストが0の辺は無条件で先頭から詰める
	int count=0;//コストが0の辺の数をカウント
	
	for(int i=0;i<edge.size();i++){
		int j = GetRand(edge.size()-1);
		tEdge t = edge[i];
		edge[i] = edge[j];
		edge[j] = t;
	}
	for(int i=0;i<edge.size();i++){
		if(edge[i].cost == 0){
			tEdge t     = edge[count];
			edge[count] = edge[i];
			edge[i]     = t;
			count++;
		}
	}
}

/*  tEdge.src.num・tEdge.dst.num と tVer.numとの対応付け  */
void associateEdgeNumber(std::vector<tVer> &ver, std::vector<tEdge> &edge){
	for(int i=0;i<edge.size();i++){
		for(int j=0;j<ver.size();j++){
			//edgeのsrcの頂点がverと同じ場所ならedgeのsrcの番号をverの番号にあわせる
			if(edge[i].src.x == ver[j].x && edge[i].src.y == ver[j].y){
				edge[i].src.num = ver[j].num;
			}
			//edgeのdstの頂点がverと同じ場所ならedgeのdstの番号をverの番号にあわせる
			else if(edge[i].dst.x == ver[j].x && edge[i].dst.y == ver[j].y){
				edge[i].dst.num = ver[j].num;
			}
		}
	}
}

//マウスのイベント処理
tMouse mouseEvent(tMouse d){
    int MouseInput;
    int MouseWheelRotVol;
    int x,y;
	int mouse_x, mouse_y;
	static int mouse_old_x=99999, mouse_old_y;

	if(mouse_old_x == 99999){//最初だけ実行したいところ
		GetMousePoint(&mouse_old_x, &mouse_old_y);
		return d;
	}

	GetMousePoint(&mouse_x, &mouse_y);

    MouseInput = GetMouseInput();
    if(( MouseInput & MOUSE_INPUT_LEFT ) != 0 ){
        d.x += mouse_x - mouse_old_x;
        d.y += mouse_y - mouse_old_y;
    }
	//ブロックサイズを大きく(小さく)したときの描画位置の計算
	x = (-d.x + mouse_x) / d.size;
	y = (-d.y + mouse_y) / d.size;
    MouseWheelRotVol = GetMouseWheelRotVol();
    if(MouseWheelRotVol > 0){
		d.size++;
        d.x -= x;
        d.y -= y;
    }
	else if(MouseWheelRotVol < 0 && d.size > 1){
		d.size--;
        d.x += x;
        d.y += y;
    }

	//mouse_oldの更新
	mouse_old_x = mouse_x;
	mouse_old_y = mouse_y;

	return d;
}
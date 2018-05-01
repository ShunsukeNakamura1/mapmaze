//地図から迷路を作りまーす
//スタート・ゴールを実装
//width,heightをXY sizeへ変更

#include "DxLib.h"
#include <stack>
#include <vector>
#include <shobjidl.h>
#include <locale.h>
#include <time.h>

#define MAPSIZE 1500//最大マップサイズ
#define WALL 0      //壁
#define ROAD 1      //道
#define VERTEX 2    //頂点
#define ANSWER 3    //回答道
#define WINDOWX 100//ウィンドウサイズ
#define WINDOWY 100
#define SMALL 500
#define CUTCOUNT 200//道の接続を切る回数

#define succeeded(hr) SUCCEEDED(hr)

double time_time;

//こーぞーたい
struct tMouse{ //マウスイベントのための構造体
	int x;		//x方向のずれ
	int y;		//y方向のずれ
	int size;	//マップのサイズ
	tMouse(int x, int y, int size){
		this->x = x;
		this->y = y;
		this->size = size;
	}
};
struct XY{
	int x;
	int y;
	XY(){}
	XY(int x, int y){
		this->x = x;
		this->y = y;
	}
};
struct tStartGoal{
	int x;
	int y;
	enum types{
		vertex,
		edge
	};
	types type;

	tStartGoal(){}
	tStartGoal(int x, int y){
		this->x = x;
		this->y = y;
	}
	tStartGoal(int x, int y, types type){
		this->x = x;
		this->y = y;
		this->type = type;
	}
};
struct tVer{  //頂点
	int num;  //頂点番号
	int x;    //x座標
	int y;    //y座標
	tVer(){}
	tVer(int x, int y){
		this->x = x;
		this->y = y;
	}
	tVer(int num, int x, int y){
		this->num = num;
		this->x = x;
		this->y = y;
	}
	
};
struct tEdge{  //辺
	tVer src;  //頂点1
	tVer dst;  //頂点2
	int cost;
	//辺探索用(頂点srcから頂点dstがどの方向にあるか)
	//  2
	//1 ! 3
	//  0
	int angle; 
	tEdge(){}
	tEdge(tVer src, tVer dst,int cost, int angle){
		this->src = src;
		this->dst = dst;
		this->cost = cost;
		this->angle = angle;
	}
};
/*  ユニオンファインド 参考:http://dai1741.github.io/maximum-algo-2012/docs/minimum-spanning-tree/  */
struct UnionFind{
	// par[i]：データiが属する木の親の番号。i == par[i]のとき、データiは木の根ノードである
	std::vector<int> par;
	// sizes[i]：根ノードiの木に含まれるデータの数。iが根ノードでない場合は無意味な値となる
	std::vector<int> sizes;

	UnionFind(int n) : par(n), sizes(n, 1) {
		// 最初は全てのデータiがグループiに存在するものとして初期化
		for (int i=0;i<n;i++){
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

//関数共
/*  画像nameを読み込んでmap[][]に二値化して保存  */
void loadMap(int **map, XY &size);
/*  地図の描画  */
void drawMap(int **map, XY size, tMouse d);
/*  ラベル番号を指定した地図の描画  */
void drawMap_label(int **map, XY size, tMouse d, int label);
/*  頂点番号を指定した地図の描画  */
void drawMap_ver(int **map, XY size, tMouse d, tVer ver);
/*  辺番号を指定した地図の描画  */
void drawMap_edge(int **map, XY size, tMouse d, tEdge edge);
/*  回答パスも示した地図の描画  */
void drawMap_answer(int **map, XY size, tMouse d);
/*  スタートとゴールの描画  */
void drawStartGoal(tStartGoal start, tStartGoal goal, tMouse d);
/*  画面の描画・保存  */
void saveScreen(int **map, XY size, tMouse d, char *filename);
/*  マウスのイベント処理  */
tMouse mouseEvent(tMouse d);
/*  細線化  */
void thinning(int **map, XY size);
int N(int *v);
int S(int *v);
/*  ラベリング 戻り値はラベル数  */
int labeling(int **map, XY size, int *label_area);
void search(int **map,int map_label[][MAPSIZE], XY size, int x, int y, int label, int *label_area);
/*  小領域をつなげる  */
void connectSmallArea(int **map, XY size, int *label_area);
/*  小領域の削除  */
void deleteSmallArea(int **map, XY size, int max_labelnum);
bool search2(int **map, XY size, int startx, int starty, int goalx, int goaly);
/*  迷路作製1(ラベリングを使用)  */
void makeMaze1(int **map, XY size, int cut_count, int *label_area);
/*  迷路作製2(探索を使用)  */
void makeMaze2(int **map, XY size, int cut_count);
/*  迷路作製3(グラフを使用)  */
void makeMaze3(int **map, XY size, int vernum, std::vector<tEdge> &edge, tMouse d);
/*  頂点(交差点)登録 戻り値は頂点数  */
int entryVer(int **map, XY size, std::vector<tVer> &ver, tStartGoal start, tStartGoal goal);
int countAround(int **map, XY size, int x, int y);
/*  辺登録  */
int entryEdge(int **map, XY size, std::vector<tEdge> &edge, std::vector<tVer> &ver);
/*  辺の削除  */
bool deleteEdge(int **map, XY size, tEdge edge);
/*  tEdgeのvector配列のシャッフル  */
void shuffleEdge(std::vector<tEdge> &edge);
/*  tEdge.src.num・tEdge.dst.num と tVer.numとの対応付け  */
void associateEdgeNumber(std::vector<tVer> &ver, std::vector<tEdge> &edge);
/*  スタート・ゴールの設置戻り値はそれぞれの座標  */
tStartGoal setStart(int **map, XY size);
tStartGoal setGoal(int **map, XY size, tStartGoal start);
/*  探索  */
bool tansaku(int **map, XY &size, XY &position, tStartGoal &start, tStartGoal &goal);
/*  メッセージの表示  */
void drawMessage(char *message);


int WINAPI WinMain( HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow )
{
	//各種設定
	SetGraphMode(WINDOWX, WINDOWY, 32);
    ChangeWindowMode( TRUE );
	SetAlwaysRunFlag(TRUE);

    // ＤＸライブラリの初期化
    if( DxLib_Init() < 0 ){
		return -1;
	}

	//変数共
	static int map[MAPSIZE][MAPSIZE];//地図のほぞん先
	int *map_address[MAPSIZE];		 //その名の通り
	XY size;						 //地図の幅
	XY save_size;					 //保存用の地図の幅(size.xやsize.yとMAPSIZEのどちらかの小さい方)
	tMouse d(0,0,1);	            //mapの中身を表示する際の情報
	tMouse d_old = d;
	int dispmode=0;                  //描画モード

	int labelnum;                    //総ラベル数
	int watch_label = 1;             //見たいラベル番号
	int label_area[10000]={0};       //各ラベルの面積

	int vernum;                      //頂点の数
	std::vector<tVer> ver;           //頂点登録用
	int watch_ver = 0;               //見たい頂点番号

	int edgenum;                     //辺の数
	std::vector<tEdge> edge;         //辺登録用
	int watch_edge = 0;              //見たい辺番号

	tStartGoal start,goal;			 //スタートとゴール

	for(int i=0;i<MAPSIZE;i++){
		map_address[i] = map[i];
	}

	//****************  前処理ここから   ****************//

    //マップ読み込み(二値化)
	loadMap(map_address, size);
	if(size.x > WINDOWX || size.y > WINDOWY){
		SetGraphMode(size.x, size.y, 32);
	}
	//保存領域の設定
	//800 * 800位が限界っぽい
	if(size.x < MAPSIZE){
		save_size.x =size.x;
	}else{
		save_size.x = MAPSIZE;
	}
	if(size.y < MAPSIZE){
		save_size.y = size.y;
	}else{
		save_size.y = MAPSIZE;
	}
	//800 * 800位が限界っぽいので・・・
	if(save_size.x > 800) save_size.x = 800;
	if(save_size.y > 800) save_size.y = 800;

	//描画して保存
	//saveScreen(map_address, save_size, d, "二値化.bmp");

	//細線化
	thinning(map_address, size);

    //描画して保存
	//saveScreen(map_address, save_size, d, "細線化.bmp");

	//ラベリング
	labelnum = labeling(map_address, size, label_area);

	//小領域をつなげYo!
	connectSmallArea(map_address, size, label_area);
	for(int i = 0; i < size.y && i < MAPSIZE; i ++ ){
		for(int j = 0; j < size.x && j < MAPSIZE; j ++ ){
			if(map[j][i] > ROAD){
				map[j][i] = ROAD;
			}
		}
	}
	
	//描画して保存
	//saveScreen(map_address, save_size, d, "小領域接続.bmp");

	thinning(map_address, size);
	labelnum = labeling(map_address, size, label_area);
	int max_labelnum;
	int max=0;
	for(int i=0;i<labelnum;i++){
		if(label_area[i+1] > max){
			max = label_area[i+1];
			max_labelnum = i+1;
		}
	}
	//小領域を削除しYo!
	deleteSmallArea(map_address, size, max_labelnum);
	for(int i = 0; i < size.y && i < MAPSIZE; i ++ ){
		for(int j = 0; j < size.x && j < MAPSIZE; j ++ ){
			if(map[j][i] > ROAD){
				map[j][i] = ROAD;
			}
		}
	}
	
	//描画して保存
	//saveScreen(map_address, save_size, d, "小領域削除.bmp");

	labelnum = labeling(map_address, size, label_area);
	
	if(labelnum == 0){
		drawMessage("Error:ラベルが0");
		WaitKey();
		exit(1);
	}
	if(labelnum > 1){
		drawMessage("ラベルが2以上");
		WaitKey();
		exit(1);
	}
	//スタート・ゴールの設置
	start = setStart(map_address, size);
	goal =  setGoal(map_address, size, start);

	//時間計測開始
	clock_t time_start = clock();

	//頂点の登録
	drawMessage("頂点登録");
	vernum = entryVer(map_address, size, ver, start, goal);
	//辺の登録
	drawMessage("辺登録");
	edgenum = entryEdge(map_address, size, edge, ver);
	//edgeに登録した辺のsrc・dstの番号をverに登録してある番号にあわせる
	associateEdgeNumber(ver, edge);
	shuffleEdge(edge);
	shuffleEdge(edge);
	mouseEvent(d);
	//****************  前処理ここまで   ****************//

	//****************  迷路作製ここから  ****************//
	//makeMaze1(map_address, size, CUTCOUNT, label_area);
	//makeMaze2(map_address, size, CUTCOUNT);
	makeMaze3(map_address, size, vernum, edge, d);
	//****************  迷路作製ここまで  ****************//

	//唐突にマウスイベントを実行
	mouseEvent(d);
	//頂点の登録
	//vernum = entryVer(map_address, size, ver, start, goal);
	//辺の登録
	//edgenum = entryEdge(map_address, size, edge, ver);
	//探索
	/*drawMessage("探索開始");
	if(tansaku(map_address, size, XY(start.x, start.y), start,goal) == false){
		drawMessage("探索失敗");
		WaitKey();
		exit(1);
	}*/
	//drawMessage("探索終了");

	//時間計測終了
	clock_t time_end = clock();
	time_time = (double)(time_end - time_start);
	//**************************  ここから表示  **************************//
	bool isdraw = true; //前のループから何か設定の変更があれば描画をする
	bool ishide = false;//文字を隠すか
	while (ProcessMessage() == 0 && CheckHitKey(KEY_INPUT_ESCAPE) == 0){
		clsDx();
		d = mouseEvent(d);   //マウスのイベント処理
		//前ループから何か変化があったら
		if(d.x != d_old.x || d.y != d_old.y || d.size != d_old.size || isdraw){
			clsDx();
			ClearDrawScreen();
			//マップの描画
			switch(dispmode){
			case 0://マップの描画
				drawMap(map_address, size, d);
				break;
			case 1://ラベルを指定したマップの描画
				drawMap_label(map_address, size, d, watch_label);
				break;
			case 2://頂点を指定したマップの描画
				drawMap_ver(map_address, size, d, ver[watch_ver]);
				break;
			case 3://辺番号を指定したマップの描画
				drawMap_edge(map_address, size, d, edge[watch_edge]);
				break;
			case 4://回答パスも示したマップの描画
				drawMap_answer(map_address, size, d);
				break;
			}
			//スタート・ゴールの描画
			drawStartGoal(start, goal, d);
			//文字の描画
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
				case 4://回答パスも指定したマップの描画
					printfDx("start  x:%3d  y:%3d\n", start.x, start.y);
					printfDx("goal   x:%3d  y:%3d\n", goal.x, goal.y);
					break;
				}
				printfDx(" a : 描画モード変更\n");
				printfDx(" b : 設定変更\n");
				printfDx(" c : 文字を隠す/表示する\n");
				printfDx(" d : 辺の削除\n");
				printfDx(" e : 画面の保存\n");
				printfDx("time : %f", time_time);
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
			printfDx(dispmode==4 ? "○" : "  ");
			printfDx(" 4 : 回答経路の表示\n");
			dispmode = KeyInputNumber( 0 , 96 , 4 , 0, FALSE);
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
			case 4:
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
			deleteEdge(map_address, size, edge[delete_edge]);
			isdraw =true;
		}
		//画面の保存
		else if(CheckHitKey( KEY_INPUT_E ) == 1){
			WaitTimer(100);
			saveScreen(map_address, save_size, d, "迷路.bmp");
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
void loadMap(int **map, XY &size){
	int handle, r, g, b, a;
	char filename[1000];

	//ダイアログボックスからファイル名を取得
	HRESULT hr = CoInitializeEx(NULL, COINIT_APARTMENTTHREADED | COINIT_DISABLE_OLE1DDE);
	if(succeeded(hr)){
		IFileOpenDialog *pFileOpen;
		//FileopenDialogオブジェクト作成
		hr = CoCreateInstance(CLSID_FileOpenDialog, NULL, CLSCTX_ALL, IID_IFileOpenDialog, reinterpret_cast<void**>(&pFileOpen));

		if(succeeded(hr)){
			//ファイルを開く・ダイアログボックスを表示する
			hr = pFileOpen->Show(NULL);

			//ダイアログボックスからファイル名を取得する
			if(succeeded(hr)){
				IShellItem *pItem;
				hr = pFileOpen->GetResult(&pItem);
				if(succeeded(hr)){
					//読める形に変換してfilepathに入れる
					PWSTR filepath;
					hr = pItem->GetDisplayName(SIGDN_FILESYSPATH, &filepath);
					//filepath(PWSTR)をfilename(char[])に変換
					if(succeeded(hr)){
						size_t wLen = 0;
						errno_t err = 0;
						setlocale(LC_ALL, "japanese");
						err = wcstombs_s(&wLen, filename, sizeof(filename), filepath, _TRUNCATE);
						CoTaskMemFree(filepath);
					}
					pItem->Release();
				}
			}
			pFileOpen->Release();
		}
		CoUninitialize();
	}

	handle = LoadSoftImage(filename);
	GetSoftImageSize( handle, &size.x, &size.y );
	for(int i = 0; i < size.y && i < MAPSIZE; i ++ ){
		for(int j = 0; j < size.x && j < MAPSIZE; j ++ ){
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
void thinning(int **map, XY size){
	bool complete = false; //更新が行われたらfalseになるtrueになれば細線化終了
	int count=0;
	while(!complete){
		complete = true;
		int v[9];

		for(int i = 1; i < size.y-1 && i < MAPSIZE-1; i ++ ){
			for(int j = 1; j < size.x-1 && j < MAPSIZE-1; j ++ ){
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



int labeling(int **map, XY size, int *label_area){
	static int map_label[MAPSIZE][MAPSIZE]={0}; //ラベリング用のマップ
	int labelnum=0;	//ラベル数 これを返す

	for(int i = 0; i < size.y && i < MAPSIZE; i ++ ){
		for(int j = 0; j < size.x && j < MAPSIZE; j ++ ){
			map_label[j][i] = 0;
		}
	}
	for(int i = 0; i < size.y && i < MAPSIZE; i ++ ){
		for(int j = 0; j < size.x && j < MAPSIZE; j ++ ){
			if(map[j][i] == ROAD && map_label[j][i] == 0){
				search(map, map_label, size, j, i, ++labelnum, label_area);
			}
		}
	}
	for(int i = 0; i < size.y && i < MAPSIZE; i ++ ){
		for(int j = 0; j < size.x && j < MAPSIZE; j ++ ){
			map[j][i] = map_label[j][i];
		}
	}
	return labelnum;
}

void search(int **map,int map_label[][MAPSIZE], XY size, int x, int y, int label, int *label_area){
	//ラベリング用の探索 藤原さん作
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
		if (px - 1 >= 0     && map_label[px-1][py] == 0 && map[px-1][py] == ROAD) p.push(std::make_pair(px - 1, py));
		if (px + 1 < size.x && map_label[px+1][py] == 0 && map[px+1][py] == ROAD) p.push(std::make_pair(px + 1, py));
		if (py - 1 >= 0     && map_label[px][py-1] == 0 && map[px][py-1] == ROAD) p.push(std::make_pair(px, py - 1));
		if (py + 1 < size.y && map_label[px][py+1] == 0 && map[px][py+1] == ROAD) p.push(std::make_pair(px, py + 1));
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
void connectSmallArea(int **map, XY size, int *label_area){
	for(int i = 0; i < size.y && i < MAPSIZE; i ++ ){
		for(int j = 0; j < size.x && j < MAPSIZE; j ++ ){
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
void deleteSmallArea(int **map, XY size, int max_labelnum){
	for(int i = 0; i < size.y && i < MAPSIZE; i ++ ){
		for(int j = 0; j < size.x && j < MAPSIZE; j ++ ){
			if(map[j][i] != max_labelnum){
				map[j][i] = WALL;
			}
		}	
	}
}

/*  迷路作製1(ラベリングを使用)  */
void makeMaze1(int **map, XY size, int cut_count, int *label_area){
	//XY a[10];
	int count = 0;
	while(cut_count != 0){
		//ランダムに選んだ点を黒画素に変える
		/*
		for(int i=0;i<10;i++){
			int x = GetRand(min(size.x,MAPSIZE));
			int y = GetRand(min(size.y,MAPSIZE));
			if(map[x][y] == WALL){
				i--;
				continue;
			}
			map[x][y] = WALL;
			a[i].x = x; a[i].y = y;
		}
		*/
		///*
		for(int i = 0; i < size.y && i < MAPSIZE; i ++ ){
			for(int j = 0; j < size.x && j < MAPSIZE; j ++ ){
				if(map[j][i] > ROAD){
					map[j][i] = ROAD;
				}
			}
		}
		int x = GetRand(min(size.x,MAPSIZE))+1;
		int y = GetRand(min(size.y,MAPSIZE))+1;
		if(map[x][y] == WALL){
			continue;
		}
		map[x][y] = WALL;
		//*/
		//ラベリングを使った連結性確認法
		//ClearDrawScreen();
		//printfDx("making… \n");
		//ScreenFlip();
		if(labeling(map, size, label_area) == 1){
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
void makeMaze2(int **map, XY size, int cut_count){
	while(cut_count != 0){
		//ランダムに選んだ点を黒画素に変える
		
		int x = GetRand(min(size.x,MAPSIZE))+1;
		int y = GetRand(min(size.y,MAPSIZE))+1;
		if(map[x][y] == WALL)continue;
		map[x][y] = WALL;
		//**********    探索を使った連結性確認法    **********//
		//  ☆
		//☆■□
		//  □
		if(map[x-1][y] == ROAD && map[x][y-1] == ROAD){
			if(search2(map, size, x-1, y, x, y-1)){}
			else{
				map[x][y]=ROAD;
				continue;
			}
		}
		//  □
		//☆■☆
		//  □
		if(map[x-1][y] == ROAD && map[x+1][y] == ROAD){
			if(search2(map, size, x-1, y, x+1, y)){}
			else{
				map[x][y]=ROAD;
				continue;
			}
		}
		//  □
		//☆■□
		//  ☆
		if(map[x-1][y] == ROAD && map[x][y+1] == ROAD){
			if(search2(map, size, x-1, y, x, y+1)){}
			else{
				map[x][y]=ROAD;
				continue;
			}
		}
		//  ☆
		//□■☆
		//  □
		if(map[x][y-1] == ROAD && map[x+1][y] == ROAD){
			if(search2(map, size, x, y-1, x+1, y)){}
			else{
				map[x][y]=ROAD;
				continue;
			}
		}
		//  ☆
		//□■□
		//  ☆
		if(map[x][y-1] == ROAD && map[x][y+1] == ROAD){
			if(search2(map, size, x, y-1, x, y+1)){}
			else{
				map[x][y]=ROAD;
				continue;
			}
		}
		//  □
		//□■☆
		//  ☆
		if(map[x+1][y] == ROAD && map[x][y+1] == ROAD){
			if(search2(map, size, x+1, y, x, y+1)){}
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
void makeMaze3(int **map, XY size, int vernum, std::vector<tEdge> &edge, tMouse d){
	UnionFind uf(vernum);
	drawMessage("Map making...\n");
	int count=0;
	for(int i=0;i<edge.size();i++){
		//srcとdstが同じグループに属していなければその辺は削除せずにsrcとdstを同じグループにする
		if(!uf.same(edge[i].src.num, edge[i].dst.num)){
			uf.unite(edge[i].src.num, edge[i].dst.num);
		}
		//srcとdstが同じグループに属していたらその辺を削除
		else{
			count++;
			deleteEdge(map, size, edge[i]);
		}
	}
	drawMessage("Map making end!!!!\n");
	WaitTimer(1000);
}

/*  連結しているかの確認  */
bool search2(int **map, XY size, int startx, int starty, int goalx, int goaly){
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
		if (px + 1 < size.x && map_flag[px+1][py] == 0 && map[px+1][py] == ROAD) p.push(std::make_pair(px + 1, py));
		if (py - 1 >= 0     && map_flag[px][py-1] == 0 && map[px][py-1] == ROAD) p.push(std::make_pair(px, py - 1));
		if (py + 1 < size.y && map_flag[px][py+1] == 0 && map[px][py+1] == ROAD) p.push(std::make_pair(px, py + 1));
	}
	return false;
}

/*  地図の描画  */
void drawMap(int **map, XY size, tMouse d){
	static const unsigned int white =GetColor(255, 255, 255);
	int x, y;

	for(int i = 0; i < size.y && i < MAPSIZE; i ++ ){
		for(int j = 0; j < size.x && j < MAPSIZE; j ++ ){
			x = j * d.size + d.x;
			y = i * d.size + d.y;
			if(map[j][i] == ROAD || map[j][i] == ANSWER){
				DrawBox(x, y, x + d.size, y + d.size, white, TRUE);
			}else if(map[j][i] == WALL){}
		}
	}
}

/*  ラベル番号を指定した地図の描画  */
void drawMap_label(int **map, XY size, tMouse d, int label){
	static const unsigned int white = GetColor(255, 255, 255);
	static const unsigned int red = GetColor(255, 0, 0);
	static const unsigned int blue = GetColor(0, 0, 255);
	int x, y;

	for(int i = 0; i < size.y && i < MAPSIZE; i ++ ){
		for(int j = 0; j < size.x && j < MAPSIZE; j ++ ){
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
	for(int i = 1; i < size.y && i < MAPSIZE; i ++ ){
		for(int j = 1; j < size.x && j < MAPSIZE; j ++ ){
			if(map[j][i] == ROAD && countAround(map, size, j, i) >= 3){
				x = j * d.size + d.x;
				y = i * d.size + d.y;
				DrawBox(x, y, x + d.size, y + d.size, white, TRUE);
			}
		}
	}
}

/*  頂点番号を指定した地図の描画  */
void drawMap_ver(int **map, XY size, tMouse d, tVer ver){
	static const unsigned int white = GetColor(255, 255, 255);
	static const unsigned int red = GetColor(255, 0, 0);
	static const unsigned int blue = GetColor(0, 0, 255);
	int x, y;

	for(int i = 0; i < size.y && i < MAPSIZE; i ++ ){
		for(int j = 0; j < size.x && j < MAPSIZE; j ++ ){
			x = j * d.size + d.x;
			y = i * d.size + d.y;
			if(map[j][i] == WALL){}
			else if(map[j][i] == ROAD || map[j][i] == ANSWER){
				DrawBox(x, y, x + d.size, y + d.size, white, TRUE);
			}
		}
	}
	x = ver.x * d.size + d.x;
	y = ver.y * d.size + d.y;
	DrawBox(x, y, x + d.size, y + d.size, red, TRUE);
}

/*  辺番号を指定した地図の描画  */
void drawMap_edge(int **map, XY size, tMouse d, tEdge edge){
	static const unsigned int white = GetColor(255, 255, 255);
	static const unsigned int red   = GetColor(255,   0,   0);
	static const unsigned int green = GetColor(  0, 255,   0);
	static const unsigned int blue  = GetColor(  0,   0, 255);
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

	for(int i = 0; i < size.y && i < MAPSIZE; i ++ ){
		for(int j = 0; j < size.x && j < MAPSIZE; j ++ ){
			x = j * d.size + d.x;
			y = i * d.size + d.y;
			if(map[j][i] == WALL){}
			else if(map[j][i] == ROAD || map[j][i] == ANSWER){
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
			if (px + 1 < size.x && map[px+1][py] != WALL && map_flag[px+1][py] == 0) p.push(std::make_pair(px + 1, py));
			if (py - 1 >= 0     && map[px][py-1] != WALL && map_flag[px][py-1] == 0) p.push(std::make_pair(px, py - 1));
			if (py + 1 < size.y && map[px][py+1] != WALL && map_flag[px][py+1] == 0) p.push(std::make_pair(px, py + 1));
		}
	}
}

/*  回答パスも示した地図の描画  */
void drawMap_answer(int **map, XY size, tMouse d){
	static const unsigned int white = GetColor(255, 255, 255);
	static const unsigned int red   = GetColor(255,   0,   0);
	int x, y;

	for(int i = 0; i < size.y && i < MAPSIZE; i ++ ){
		for(int j = 0; j < size.x && j < MAPSIZE; j ++ ){
			x = j * d.size + d.x;
			y = i * d.size + d.y;
			//道は白，回答パスは赤で表示
			if(map[j][i] == ROAD){
				DrawBox(x, y, x + d.size, y + d.size, white, TRUE);
			}else if(map[j][i] == WALL){}
			else if(map[j][i] == ANSWER){
				DrawBox(x, y, x + d.size, y + d.size, red, TRUE);
			}
		}
	}
}

/*  スタートとゴールの描画  */
void drawStartGoal(tStartGoal start, tStartGoal goal, tMouse d){
	int x,y;
	const unsigned int red  = GetColor(255,   0,   0);
	const unsigned int green= GetColor(  0, 255,   0);
	//スタート描画
	x = start.x * d.size + d.x;
	y = start.y * d.size + d.y;
	DrawBox(x, y, x + d.size, y + d.size, green, TRUE);
	//ゴール描画
	x = goal.x * d.size + d.x;
	y = goal.y * d.size + d.y;
	DrawBox(x, y, x + d.size, y + d.size, red, TRUE);
}

/*  画面の描画・保存  */
void saveScreen(int **map, XY size, tMouse d, char *filename){
	ClearDrawScreen();
	drawMap(map, size, d);
	ScreenFlip();
	SaveDrawScreen( 0, 0, size.x, size.y, filename);
}

/*  頂点(交差点)登録 戻り値は頂点数  */
int entryVer(int **map, XY size, std::vector<tVer> &ver, tStartGoal start, tStartGoal goal){
	int ver_count = 0;
	for(int i = 1; i < size.y && i < MAPSIZE; i ++ ){
		for(int j = 1; j < size.x && j < MAPSIZE; j ++ ){
			if(map[j][i] == ROAD && countAround(map, size, j, i) >= 3){
				ver.push_back(tVer(ver_count++, j, i));
			}
		}
	}
	//スタートとゴールが辺ならその部分を頂点として登録(頂点は迷路作製の時に消されないため)
	if(start.type == tStartGoal::edge){
		ver.push_back(tVer(ver_count++, start.x, start.y));
	}
	if(goal.type == tStartGoal::edge){
		ver.push_back(tVer(ver_count++,  goal.x,  goal.y));
	}
	return ver_count;
}
int countAround(int **map, XY size, int x, int y){
	int count = 0;
	if(x > 0      && map[x-1][ y ] == ROAD) count++;
	if(x < size.x && map[x+1][ y ] == ROAD) count++;
	if(y > 0      && map[ x ][y-1] == ROAD) count++;
	if(y < size.y && map[ x ][y+1] == ROAD) count++;
	return count;
}

/*  辺の登録  */
int entryEdge(int **map, XY size, std::vector<tEdge> &edge, std::vector<tVer> &ver){
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
	for(int i = 0; i < size.y && i < MAPSIZE; i ++ ){
		for(int j = 0; j < size.x && j < MAPSIZE; j ++ ){
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
			if(ver[i].x+dx[j] < 0 || ver[i].y+dy[j] < 0 || ver[i].x+dx[j] >= size.x || ver[i].y+dy[j] >= size.y){
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
				if (px + 1 < size.x && mapcopy[px+1][py] != WALL && (px+1 != ver[i].x || py != ver[i].y)){
					p.push(std::make_pair(px + 1, py));
				}
				if (py - 1 >= 0     && mapcopy[px][py-1] != WALL && (px != ver[i].x || py-1 != ver[i].y)){
					p.push(std::make_pair(px, py - 1));
				}
				if (py + 1 < size.y && mapcopy[px][py+1] != WALL && (px != ver[i].x || py+1 != ver[i].y)){
					p.push(std::make_pair(px, py + 1));
				}
			}
		}
	}
	return edgecount;
}

/*  辺の削除  */
bool deleteEdge(int **map, XY size, tEdge edge){
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
			if (px + 1 < size.x && map[px+1][py] != WALL && map_flag[px+1][py] == 0) p.push(std::make_pair(px + 1, py));
			if (py - 1 >= 0     && map[px][py-1] != WALL && map_flag[px][py-1] == 0) p.push(std::make_pair(px, py - 1));
			if (py + 1 < size.y && map[px][py+1] != WALL && map_flag[px][py+1] == 0) p.push(std::make_pair(px, py + 1));
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

/*  スタートの設定  */
tStartGoal setStart(int **map, XY size){
	const unsigned int white = GetColor(255,255,255);
	const unsigned int green = GetColor(0,255,0);
	int x,y;
	XY mouse;
	tStartGoal start(0,0);
	tMouse d(0, 0, 1); //mapの中身を表示する際の情報
	tMouse d_old = d;
	bool wait = false;

	while(ProcessMessage() == 0 && CheckHitKey(KEY_INPUT_ESCAPE) == 0){
		d = mouseEvent(d);
		clsDx();
		ClearDrawScreen();
		printfDx("x:%d  y:%d\n",start.x,start.y);
		printfDx("スペースキーで決定\n");
		drawMap(map, size, d);
		drawStartGoal(start, tStartGoal(0,0), d);
		x = start.x * d.size + d.x;
		y = start.y * d.size + d.y;
		///DrawBox(x, y, x + d.size, y + d.size, green, TRUE);
		
		if(CheckHitKey( KEY_INPUT_UP ) == 1 && start.y > 0){
			start.y--;
			wait = true;
		}if(CheckHitKey( KEY_INPUT_DOWN ) == 1 && start.y < size.y){
			start.y++;
			wait = true;
		}if(CheckHitKey( KEY_INPUT_LEFT ) == 1 && start.x > 0){
			start.x--;
			wait = true;
		}if(CheckHitKey( KEY_INPUT_RIGHT ) == 1 && start.x < size.x){
			start.x++;
			wait = true;
		}if(CheckHitKey( KEY_INPUT_SPACE ) == 1 && map[start.x][start.y] == ROAD){
			break;
		}
		if(wait){
			if(CheckHitKey( KEY_INPUT_A ) == 1){
				WaitTimer(10);
			}else{
				WaitTimer(100);
			}
			wait = false;
		}
		d_old = d;
	}
	//頂点なら頂点として，違えば辺として登録
	if(countAround(map, size, start.x, start.y) >= 3){
		start.type = tStartGoal::vertex;
	}else{
		start.type = tStartGoal::edge;
	}
	return start;
}
/*  ゴールの設定  */
tStartGoal setGoal(int **map, XY size, tStartGoal start){
	const unsigned int white = GetColor(255,255,255);
	const unsigned int green = GetColor(0,255,0);
	int x,y;
	XY mouse;
	tStartGoal goal(size.x,size.y);
	tMouse d(0, 0, 1); //mapの中身を表示する際の情報
	tMouse d_old = d;
	bool wait = false;

	while(ProcessMessage() == 0 && CheckHitKey(KEY_INPUT_ESCAPE) == 0){
		d = mouseEvent(d);
		clsDx();
		ClearDrawScreen();
		printfDx("x:%d  y:%d\n",goal.x, goal.y);
		printfDx("スペースキーで決定\n");
		drawMap(map, size, d);
		drawStartGoal(start, goal, d);
		x = start.x * d.size + d.x;
		y = start.y * d.size + d.y;
		//DrawBox(x, y, x + d.size, y + d.size, green, TRUE);
		
		if(CheckHitKey( KEY_INPUT_UP ) == 1 && goal.y > 0){
			goal.y--;
			wait = true;
		}if(CheckHitKey( KEY_INPUT_DOWN ) == 1 && goal.y < size.y){
			goal.y++;
			wait = true;
		}if(CheckHitKey( KEY_INPUT_LEFT ) == 1 && goal.x > 0){
			goal.x--;
			wait = true;
		}if(CheckHitKey( KEY_INPUT_RIGHT ) == 1 && goal.x < size.x){
			goal.x++;
			wait = true;
		}if(CheckHitKey( KEY_INPUT_SPACE ) == 1 && map[goal.x][goal.y] == ROAD){
			break;
		}
		if(wait){
			if(CheckHitKey( KEY_INPUT_A ) == 1){
				WaitTimer(10);
			}else{
				WaitTimer(100);
			}
			wait = false;
		}
		d_old = d;
	}
	//頂点なら頂点として登録,違えば辺
	if(countAround(map, size, goal.x, goal.y) >= 3){
		goal.type = tStartGoal::vertex;
	}else{
		goal.type = tStartGoal::edge;
	}
	return goal;
}
/*  探索  */
bool tansaku(int **map, XY &size, XY &position, tStartGoal &start, tStartGoal &goal){
	//現在の地点を探索済みにする
	map[position.x][position.y] = ANSWER;
	//ゴールが見つかればtrueを返す
	if(position.x == goal.x && position.y == goal.y){
		return true;
	}
	//ゴールが見つからなければ四方を探索する
	//1マス先が道かつそこを探索してゴールを見つければtrueを返す
	if(position.x > 0){
		XY position2(position.x-1, position.y);
		if(map[position.x-1][position.y] == ROAD && tansaku(map, size, position2, start, goal)){
			return true;
		}
	}
	if(position.x < size.x){
		XY position2(position.x+1, position.y);
		if(map[position.x+1][position.y] == ROAD && tansaku(map, size, position2, start, goal)){
			return true;
		}
	}
	if(position.y > 0){
		XY position2(position.x, position.y-1);
		if(map[position.x][position.y-1] == ROAD && tansaku(map, size, position2, start, goal)){
			return true;
		}
	}
	if(position.y < size.y){
		XY position2(position.x, position.y+1);
		if(map[position.x][position.y+1] == ROAD && tansaku(map, size, position2, start, goal)){
			return true;
		}
	}

	//四方を探索してゴールがなかったらその道を捨ててfalseを返す
	map[position.x][position.y] = ROAD;
	return false;
}

/*  マウスのイベント処理  */
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
/*  メッセージの表示  */
void drawMessage(char *message){
	clsDx();
	ClearDrawScreen();
	printfDx(message);
	ScreenFlip();
}
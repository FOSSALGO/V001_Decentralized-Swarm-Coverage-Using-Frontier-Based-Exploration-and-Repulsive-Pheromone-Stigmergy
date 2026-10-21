package swarmdrone.decentralizedswarmcoverage_ijrcs;

import java.util.*;

/**
 * Algorithm Specification v1.1. Independent local occupancy maps, frontier
 * exploration, single-source Dijkstra, and globally shared repulsive pheromone.
 * The shared field is an ideal global communication assumption, NOT fully
 * distributed storage. Ground truth is available only to sensor/physics/metrics.
 */
public final class DecentralizedExploration {
    private final int MAX_STEPS = Integer.getInteger("swarm.maxSteps", 2500);
    private final int STALL_LIMIT = Integer.getInteger("swarm.stallLimit", 30);
    private final double LAMBDA = Double.parseDouble(System.getProperty("swarm.lambda", "1.0"));
    private final double RHO = Double.parseDouble(System.getProperty("swarm.rho", "0.05"));
    private final double DEPOSIT = Double.parseDouble(System.getProperty("swarm.deposit", "1.0"));
    private static final int[][] N8 = {{0,1},{1,1},{1,0},{1,-1},{0,-1},{-1,-1},{-1,0},{-1,1}};
    private static final double SQRT2 = Math.sqrt(2.0);
    private static final double EPS = 1e-9;
    private final int[][] truth;
    private final ArrayList<Drone> drones;
    private final int rows, cols, n;
    private final int[][][] localMaps;
    private final double[][] pheromone;
    private final int[][] firstObservedAt;
    private final int[][] visits;
    private final BitSet[] visitors;
    private final boolean[][] reachable;
    private final Point[] goals;
    private final List<Point>[] routes;
    private final int[] row, col, blockedCount, mapVersion;
    private final int[][][] exhaustedAtVersion;
    private final boolean RECOVERY = Boolean.parseBoolean(System.getProperty("swarm.recovery", "true"));
    private final String TRACE_FILE = System.getProperty("swarm.trace", "");
    private final int TRACE_LAST = Integer.getInteger("swarm.traceLast", 0);
    private final ArrayList<String> traceRows = new ArrayList<>();
    private int collisionRejects, alternateMoves, exhaustionEvents, recoveryAttempts;
    private final int reachableFree;
    private final double coverage, revisit, overlap;
    private int observedReachable, steps, uniqueVisits, totalVisits;
    private int decisions, changedDecisions, positivePathDecisions, differingPenaltyDecisions;
    private int noFrontierDecisions, successfulMoves;
    private String terminationReason = "NOT_STARTED";

    @SuppressWarnings("unchecked")
    public DecentralizedExploration(int[][] environment, ArrayList<Drone> agents) {
        if (environment == null || environment.length == 0 || environment[0].length == 0)
            throw new IllegalArgumentException("Empty environment");
        if (!Double.isFinite(LAMBDA) || LAMBDA < 0 || !Double.isFinite(RHO) || RHO < 0 || RHO > 1
                || !Double.isFinite(DEPOSIT) || DEPOSIT < 0 || MAX_STEPS <= 0 || STALL_LIMIT <= 0)
            throw new IllegalArgumentException("Invalid experiment parameters");
        truth = environment;
        drones = agents;
        rows = environment.length; cols = environment[0].length; n = agents.size();
        localMaps = new int[n][rows][cols];
        pheromone = new double[rows][cols];
        firstObservedAt = new int[rows][cols];
        visits = new int[rows][cols];
        visitors = new BitSet[rows * cols];
        reachable = new boolean[rows][cols];
        goals = new Point[n]; routes = new List[n];
        row = new int[n]; col = new int[n]; blockedCount = new int[n]; mapVersion = new int[n];
        exhaustedAtVersion = new int[n][rows][cols];
        for (int[][] m : exhaustedAtVersion) for (int[] a : m) Arrays.fill(a,-1);
        for (int[] a : firstObservedAt) Arrays.fill(a, -1);
        for (int i = 0; i < n; i++) {
            Drone d = drones.get(i);
            row[i] = d.initialRowPosition; col[i] = d.initialColPosition;
            if (!free(row[i], col[i])) throw new IllegalArgumentException("Start blocked: D"+(i+1));
            for (int j = 0; j < i; j++)
                if (row[j] == row[i] && col[j] == col[i]) throw new IllegalArgumentException("Duplicate starts");
            d.path = new ArrayList<>();
            d.path.add(new Point(row[i], col[i])); d.frameIndex = 0;
            routes[i] = new ArrayList<>();
            visit(i, row[i], col[i]);
        }
        reachableFree = markReachable();
        simulate();
        coverage = reachableFree == 0 ? 0 : 100.0 * uniqueVisits / reachableFree;
        revisit = totalVisits == 0 ? 0 : 100.0 * (totalVisits - uniqueVisits) / totalVisits;
        int overlapCells = 0;
        for (BitSet b : visitors) if (b != null && b.cardinality() > 1) overlapCells++;
        overlap = uniqueVisits == 0 ? 0 : 100.0 * overlapCells / uniqueVisits;
        System.out.printf(Locale.ROOT,
                "Dijkstra/RP lambda=%.3f: %s steps=%d observed=%.2f%% visited=%.2f%% revisit=%.2f%% overlap=%.2f%% decisions=%d changes=%d positivePaths=%d penaltySpread=%d moves=%d%n",
                LAMBDA, terminationReason, steps, getObservedRatio(), coverage, revisit, overlap,
                decisions, changedDecisions, positivePathDecisions, differingPenaltyDecisions, successfulMoves);
    }

    private boolean inside(int r, int c) { return r >= 0 && r < rows && c >= 0 && c < cols; }
    private boolean free(int r, int c) { return inside(r,c) && truth[r][c] != -1; }
    // Ground-truth legal moves: simulator physics / reachable evaluation ONLY.
    private boolean physicalLegal(int r, int c, int nr, int nc) {
        if (!free(nr,nc)) return false;
        int dr=nr-r, dc=nc-c;
        return Math.abs(dr)<=1 && Math.abs(dc)<=1 && (dr!=0 || dc!=0)
                && (dr==0 || dc==0 || (free(r,nc) && free(nr,c)));
    }
    // Navigational legality: never uses ground truth.
    private boolean knownLegal(int id, int r, int c, int nr, int nc) {
        int[][] m = localMaps[id];
        if (!inside(nr,nc) || m[nr][nc] != 1) return false;
        int dr=nr-r, dc=nc-c;
        return Math.abs(dr)<=1 && Math.abs(dc)<=1 && (dr!=0 || dc!=0)
                && (dr==0 || dc==0 || (m[r][nc]==1 && m[nr][c]==1));
    }
    private int markReachable() {
        ArrayDeque<Point> q = new ArrayDeque<>();
        for (int i=0;i<n;i++) if (!reachable[row[i]][col[i]]) {
            reachable[row[i]][col[i]]=true; q.add(new Point(row[i],col[i]));
        }
        int count=0;
        while (!q.isEmpty()) {
            Point p=q.poll(); count++;
            for (int[] d:N8) {
                int nr=p.row()+d[0], nc=p.col()+d[1];
                if (physicalLegal(p.row(),p.col(),nr,nc) && !reachable[nr][nc]) {
                    reachable[nr][nc]=true; q.add(new Point(nr,nc));
                }
            }
        }
        return count;
    }
    private void visit(int id,int r,int c) {
        totalVisits++;
        if (visits[r][c]++ == 0) uniqueVisits++;
        int k=r*cols+c;
        if (visitors[k]==null) visitors[k]=new BitSet(n);
        visitors[k].set(id);
    }
    private void discover(int id,int r,int c,int t) {
        if (!inside(r,c)) return;
        if (localMaps[id][r][c] != 0) return;
        localMaps[id][r][c]=truth[r][c]==-1 ? -1 : 1;
        mapVersion[id]++;
        if (localMaps[id][r][c]==1 && reachable[r][c] && firstObservedAt[r][c]==-1) {
            firstObservedAt[r][c]=t; observedReachable++;
        }
    }
    // Original 360 degree raycasting stepping kept unchanged except target map.
    private void sense(int id,int radius,int t) {
        int r=row[id],c=col[id];
        discover(id,r,c,t);
        for (int degree=0;degree<360;degree++) {
            double theta=Math.toRadians(degree),dr=Math.sin(theta),dc=Math.cos(theta);
            double x=c+0.5,y=r+0.5;
            int prev=-1;
            for (double distance=0.125;distance<=radius+0.5;distance+=0.125) {
                int rr=(int)Math.floor(y+dr*distance),cc=(int)Math.floor(x+dc*distance);
                if (!inside(rr,cc)) break;
                int key=rr*cols+cc;
                if (key==prev) continue;
                prev=key;
                discover(id,rr,cc,t);
                if (truth[rr][cc]==-1) break;
            }
        }
    }
    private boolean isFrontier(int id,int r,int c) {
        if (localMaps[id][r][c]!=1) return false;
        for (int[] v:N8) {
            int nr=r+v[0],nc=c+v[1];
            if (inside(nr,nc) && localMaps[id][nr][nc]==0) return true;
        }
        return false;
    }
    private static record Node(int r,int c,double dist) {}

    /** One Dijkstra source; predecessor tree is deterministic (fixed N8 order). */
    private List<Point> selectRoute(int id) {
        decisions++;
        int sr=row[id],sc=col[id];
        double[][] dist=new double[rows][cols];
        int[][] parent=new int[rows][cols];
        for (double[] a:dist) Arrays.fill(a,Double.POSITIVE_INFINITY);
        for (int[] a:parent) Arrays.fill(a,-1);
        PriorityQueue<Node> pq=new PriorityQueue<>(Comparator.comparingDouble(Node::dist)
                .thenComparingInt(Node::r).thenComparingInt(Node::c));
        dist[sr][sc]=0; pq.add(new Node(sr,sc,0));
        while (!pq.isEmpty()) {
            Node p=pq.poll();
            if (p.dist()>dist[p.r()][p.c()]+EPS) continue;
            for (int[] v:N8) {
                int nr=p.r()+v[0],nc=p.c()+v[1];
                if (!knownLegal(id,p.r(),p.c(),nr,nc)) continue;
                double nd=p.dist()+(v[0]==0||v[1]==0?1:SQRT2);
                if (nd+EPS<dist[nr][nc]) {
                    dist[nr][nc]=nd; parent[nr][nc]=p.r()*cols+p.c();
                    pq.add(new Node(nr,nc,nd));
                }
            }
        }
        Point best=null, nearest=null;
        double bestScore=Double.POSITIVE_INFINITY, nearestDistance=Double.POSITIVE_INFINITY;
        double minPenalty=Double.POSITIVE_INFINITY,maxPenalty=Double.NEGATIVE_INFINITY;
        boolean positive=false;
        for (int r=0;r<rows;r++) for (int c=0;c<cols;c++) {
            if (!isFrontier(id,r,c) || !Double.isFinite(dist[r][c]) || (r==sr && c==sc) || (RECOVERY && exhaustedAtVersion[id][r][c]==mapVersion[id])) continue;
            double penalty=0;
            int k=r*cols+c;
            while (k!=sr*cols+sc) {
                penalty+=pheromone[k/cols][k%cols];
                k=parent[k/cols][k%cols];
                if (k<0) throw new IllegalStateException("Broken Dijkstra predecessor");
            }
            positive |= penalty>EPS;
            minPenalty=Math.min(minPenalty,penalty);maxPenalty=Math.max(maxPenalty,penalty);
            Point f=new Point(r,c);
            if (dist[r][c]<nearestDistance-EPS || (Math.abs(dist[r][c]-nearestDistance)<=EPS && compare(f,nearest)<0)) {
                nearestDistance=dist[r][c];nearest=f;
            }
            double score=dist[r][c]+LAMBDA*penalty;
            if (score<bestScore-EPS || (Math.abs(score-bestScore)<=EPS &&
                    (best==null || dist[r][c]<dist[best.row()][best.col()]-EPS ||
                    (Math.abs(dist[r][c]-dist[best.row()][best.col()])<=EPS && compare(f,best)<0)))) {
                bestScore=score;best=f;
            }
        }
        if (best==null) {noFrontierDecisions++; return Collections.emptyList();}
        if (positive) positivePathDecisions++;
        if (maxPenalty-minPenalty>EPS) differingPenaltyDecisions++;
        if (!best.equals(nearest)) changedDecisions++;
        LinkedList<Point> path=new LinkedList<>();
        int k=best.row()*cols+best.col();
        while (k!=sr*cols+sc) {
            path.addFirst(new Point(k/cols,k%cols));
            k=parent[k/cols][k%cols];
        }
        path.addFirst(new Point(sr,sc));
        return path;
    }
    private static int compare(Point a,Point b) {
        if (b==null) return -1;
        int t=Integer.compare(a.row(),b.row());
        return t!=0?t:Integer.compare(a.col(),b.col());
    }

    private void simulate() {
        if (n==0) {terminationReason="NO_DRONES";return;}
        int stagnant=0;
        for (int t=0;t<MAX_STEPS;t++) {
            for (int i=0;i<n;i++) sense(i,drones.get(i).sensorFoV,t);
            if (observedReachable==reachableFree) {
                terminationReason="ALL_REACHABLE_FREE_CELLS_OBSERVED";steps=t;break;
            }
            Point[] next=new Point[n];
            Point[] desired=new Point[n];
            String[] reason=new String[n];
            Arrays.fill(reason,"OK");
            // All agents read the same pre-movement global pheromone snapshot.
            for (int i=0;i<n;i++) {
                Point goal=goals[i];
                boolean valid=goal!=null && isFrontier(i,goal.row(),goal.col()) && routes[i].size()>1;
                if (valid) {
                    Point waypoint=routes[i].get(1);
                    valid=knownLegal(i,row[i],col[i],waypoint.row(),waypoint.col());
                }
                if (!valid) {
                    if (RECOVERY && goal!=null && goal.row()==row[i] && goal.col()==col[i] &&
                            isFrontier(i,row[i],col[i])) {
                        exhaustedAtVersion[i][row[i]][col[i]]=mapVersion[i]; exhaustionEvents++;
                    }
                    routes[i]=selectRoute(i);
                    goals[i]=routes[i].isEmpty()?null:routes[i].get(routes[i].size()-1);
                }
                if (routes[i].size()>1) {
                    Point step=routes[i].get(1);
                    next[i]=knownLegal(i,row[i],col[i],step.row(),step.col())
                            ? step:new Point(row[i],col[i]);
                } else next[i]=new Point(row[i],col[i]);
                // Physics catches any non-observed corner geometry; no oracle used by planner.
                if (!physicalLegalOrStay(i,next[i])) {
                    next[i]=new Point(row[i],col[i]); routes[i]=Collections.emptyList();
                    reason[i]="PHYSICAL_BLOCK";
                }
                desired[i]=next[i];
            }
            // Deterministic simultaneous resolver. Every drone proposes autonomously.
            // Moving onto stationary drone and head-on swaps remain forbidden.
            resolveMoves(next, reason, t);
            if (RECOVERY) {
                // A blocked agent may yield to an adjacent known-free cell (not a target assignment).
                // This is common to both lambda=0 and lambda>0.
                for (int pass=0;pass<n;pass++) {
                    boolean added=false;
                    for (int i=0;i<n;i++) {
                        if (!same(next[i],row[i],col[i]) || !same(desired[i],row[i],col[i]) && blockedCount[i]<1) continue;
                        if (blockedCount[i]<1 && same(desired[i],row[i],col[i])) continue;
                        recoveryAttempts++;
                        Point alt=yieldStep(i,next,t);
                        if (alt!=null) {
                            next[i]=alt; reason[i]="YIELD"; alternateMoves++; added=true;
                            resolveMoves(next,reason,t);
                        }
                    }
                    if (!added) break;
                }
            }
            // Evaporation once per global step, then movement-based deposits.
            for (int r=0;r<rows;r++) for (int c=0;c<cols;c++) pheromone[r][c]*=(1-RHO);
            boolean moved=false;
            for (int i=0;i<n;i++) {
                int nr=next[i].row(),nc=next[i].col();
                if (nr!=row[i] || nc!=col[i]) {
                    row[i]=nr; col[i]=nc;
                    visit(i,nr,nc); moved=true;successfulMoves++;
                    pheromone[nr][nc]+=DEPOSIT;
                    blockedCount[i]=0;
                    if (routes[i].size()>1 && routes[i].get(1).row()==nr && routes[i].get(1).col()==nc)
                        routes[i].remove(0);
                } else if (routes[i].size()>1) {
                    routes[i]=Collections.emptyList(); blockedCount[i]++;
                } else if (!same(desired[i],row[i],col[i])) {
                    blockedCount[i]++;
                }
                if (TRACE_LAST!=0 || !TRACE_FILE.isBlank()) {
                    Point target=goals[i];
                    traceRows.add(String.format(Locale.ROOT,"%d,%d,%d,%d,%s,%s,%s,%s,%d,%d,%d,%d,%.6f",t,i+1,row[i],col[i],
                            target==null?"NONE":target.row()+":"+target.col(),
                            desired[i].row()+":"+desired[i].col(),next[i].row()+":"+next[i].col(),
                            reason[i],blockedCount[i],mapVersion[i],countFrontiers(i),observedReachable,
                            pheromone[row[i]][col[i]]));
                }
                drones.get(i).path.add(new Point(row[i],col[i]));
            }
            steps=t+1;
            stagnant=moved?0:stagnant+1;
            if (stagnant>=STALL_LIMIT) {terminationReason="STALL_"+STALL_LIMIT+"_STEPS";break;}
        }
        for (int i=0;i<n;i++) sense(i,drones.get(i).sensorFoV,steps);
        if (observedReachable==reachableFree) terminationReason="ALL_REACHABLE_FREE_CELLS_OBSERVED";
        if (terminationReason.equals("NOT_STARTED")) terminationReason="MAX_STEPS";
        if (!TRACE_FILE.isBlank()) writeTrace();
        System.out.printf(Locale.ROOT,"DIAGNOSTIC recovery=%s collisionRejects=%d yieldMoves=%d exhaustedFrontiers=%d recoveryAttempts=%d%n",
                RECOVERY,collisionRejects,alternateMoves,exhaustionEvents,recoveryAttempts);
        for (Drone d:drones) d.frameIndex=0;
    }
    private boolean physicalLegalOrStay(int id,Point next) {
        if (next.row()==row[id] && next.col()==col[id]) return true;
        return physicalLegal(row[id],col[id],next.row(),next.col());
    }

    private static boolean same(Point p,int r,int c){return p.row()==r && p.col()==c;}
    private int countFrontiers(int id){
        int z=0;for(int r=0;r<rows;r++)for(int c=0;c<cols;c++)if(isFrontier(id,r,c))z++;return z;
    }
    // Resolve simultaneous occupancy without changing any occupancy map or pheromone.
    private void resolveMoves(Point[] next,String[] reason,int tick){
        for(int pass=0;pass<n*n+2;pass++){
            boolean changed=false;
            for(int i=0;i<n;i++)for(int j=i+1;j<n;j++){
                if (same(next[i],next[j].row(),next[j].col())){
                    // Stationary occupant has priority; otherwise rotate winner across timesteps.
                    int loser=same(next[i],row[i],col[i])?j:
                              same(next[j],row[j],col[j])?i:((tick%2==0)?j:i);
                    if (!same(next[loser],row[loser],col[loser])) {
                        next[loser]=new Point(row[loser],col[loser]);reason[loser]="SAME_DEST";
                        collisionRejects++;changed=true;
                    }
                }
                if (same(next[i],row[j],col[j]) && same(next[j],row[i],col[i]) &&
                       !same(next[i],row[i],col[i])) {
                    next[i]=new Point(row[i],col[i]);next[j]=new Point(row[j],col[j]);
                    reason[i]="SWAP";reason[j]="SWAP";collisionRejects+=2;changed=true;
                }
                if (crossingDiagonals(i,j,next)) {
                    int loser=tick%2==0?j:i;
                    next[loser]=new Point(row[loser],col[loser]);reason[loser]="DIAGONAL_CROSS";
                    collisionRejects++;changed=true;
                }
            }
            for(int i=0;i<n;i++)for(int j=0;j<n;j++)if(i!=j &&
                    same(next[i],row[j],col[j]) && same(next[j],row[j],col[j]) &&
                    !same(next[i],row[i],col[i])) {
                next[i]=new Point(row[i],col[i]);reason[i]="OCCUPIED";collisionRejects++;changed=true;
            }
            if(!changed)break;
        }
    }
    private boolean crossingDiagonals(int i,int j,Point[] next){
        int a=next[i].row()-row[i],b=next[i].col()-col[i];
        int c=next[j].row()-row[j],d=next[j].col()-col[j];
        return Math.abs(a)==1 && Math.abs(b)==1 && Math.abs(c)==1 && Math.abs(d)==1 &&
                row[i]+next[i].row()==row[j]+next[j].row() &&
                col[i]+next[i].col()==col[j]+next[j].col();
    }
    private Point yieldStep(int id,Point[] next,int tick){
        int r=row[id],c=col[id];
        Point best=null;double bestCost=Double.POSITIVE_INFINITY;
        int start=(id+tick)%N8.length;
        for(int z=0;z<N8.length;z++){
            int[] d=N8[(start+z)%N8.length];int nr=r+d[0],nc=c+d[1];
            if(!knownLegal(id,r,c,nr,nc) || !physicalLegal(r,c,nr,nc))continue;
            boolean taken=false;
            for(int j=0;j<n;j++)if(j!=id && (same(next[j],nr,nc) ||
                    (row[j]==nr && col[j]==nc) ||
                    (same(next[j],r,c) && row[j]==nr && col[j]==nc))) {taken=true;break;}
            if(taken)continue;
            // Prefer progress toward current goal, but allow moving away to yield.
            Point goal=goals[id];
            double dist=goal==null?0:Math.hypot(nr-goal.row(),nc-goal.col());
            double cost=dist;
            if(cost<bestCost-EPS){bestCost=cost;best=new Point(nr,nc);}
        }
        return best;
    }
    private void writeTrace(){
        try {
            java.nio.file.Path out=java.nio.file.Path.of(TRACE_FILE);
            if(out.getParent()!=null)java.nio.file.Files.createDirectories(out.getParent());
            List<String> data=TRACE_LAST>0 && traceRows.size()>TRACE_LAST*n?
                    traceRows.subList(traceRows.size()-TRACE_LAST*n,traceRows.size()):traceRows;
            try(java.io.BufferedWriter w=java.nio.file.Files.newBufferedWriter(out)){
                w.write("tick,drone,row,col,goal,desired,accepted,reason,blocked_count,map_version,frontiers,observed_cells,pheromone_at_position\n");
                for(String line:data){w.write(line);w.newLine();}
            }
        }catch(java.io.IOException e){throw new java.io.UncheckedIOException(e);}
    }
    public int getCollisionRejects(){return collisionRejects;}
    public int getAlternateMoves(){return alternateMoves;}
    public int getExhaustionEvents(){return exhaustionEvents;}

    // Existing GUI API: do not rename/remove.
    public int[][] getFirstObservedAt(){return firstObservedAt;}
    public boolean[][] getMustExploreMask(){return reachable;}
    public int getSteps(){return steps;}
    public double getCoverage(){return coverage;}
    public double getRevisit(){return revisit;}
    public double getOverlap(){return overlap;}
    public double getObservedRatio(){return reachableFree==0?100:100.0*observedReachable/reachableFree;}
    public String getTerminationReason(){return terminationReason;}
    // Additive diagnostics getters (no GUI modifications).
    public int getDecisions(){return decisions;}
    public int getChangedDecisions(){return changedDecisions;}
    public int getPositivePathDecisions(){return positivePathDecisions;}
    public int getDifferingPenaltyDecisions(){return differingPenaltyDecisions;}
    public int getSuccessfulMoves(){return successfulMoves;}
    public double[][] getPheromone(){return pheromone;}
    public int[][] getLocalMap(int id){return localMaps[id];}
}

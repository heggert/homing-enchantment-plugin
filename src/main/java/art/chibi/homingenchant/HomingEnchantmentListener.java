package art.chibi.homingenchant;

import org.bukkit.FluidCollisionMode;
import org.bukkit.Location;
import org.bukkit.Material;
import org.bukkit.Particle;
import org.bukkit.util.RayTraceResult;
import org.bukkit.Registry;
import org.bukkit.World;
import org.bukkit.block.Block;
import org.bukkit.enchantments.Enchantment;
import org.bukkit.entity.Arrow;
import org.bukkit.entity.Entity;
import org.bukkit.entity.LivingEntity;
import org.bukkit.event.EventHandler;
import org.bukkit.event.Listener;
import org.bukkit.event.entity.EntityShootBowEvent;
import org.bukkit.projectiles.ProjectileSource;
import org.bukkit.scheduler.BukkitRunnable;
import org.bukkit.util.Vector;

import io.papermc.paper.registry.RegistryAccess;
import io.papermc.paper.registry.RegistryKey;

import java.util.*;

/**
 * A more optimized "military-grade" homing arrow:
 */
public class HomingEnchantmentListener implements Listener {

    // ------------------- CONFIG -------------------
    private static final int SEARCH_RADIUS = 80; // Smaller A* search radius
    private static final int MAX_EXPANSIONS = 800; // Max A* expansions before we bail
    private static final int PATHFIND_INTERVAL = 2; // Ticks between path computations
    private static final int TARGET_CHECK_INTERVAL = 2; // Ticks between target re-check
    private static final int PARTICLE_INTERVAL = 1; // Ticks between particle spawns

    private static final double CLOSE_RANGE = 15.0; // Distance that triggers speed boost
    private static final double SPEED_BOOST_FACTOR = 8.0; // How much to speed up at close range
    // ------------------------------------------------

    @EventHandler
    public void onBowShoot(EntityShootBowEvent event) {
        if (!(event.getProjectile() instanceof Arrow))
            return;
        if (event.getBow() == null)
            return;

        RegistryAccess registry = RegistryAccess.registryAccess();
        Registry<Enchantment> enchantments = registry.getRegistry(RegistryKey.ENCHANTMENT);

        // Check if the bow has the custom "chibi:aimbot" enchant
        if (!event.getBow()
                .containsEnchantment(enchantments.get(HomingEnchantmentConstants.ENCHANTMENT_KEY))) {
            return;
        }

        Arrow arrow = (Arrow) event.getProjectile();
        startOptimizedHomingTask(arrow);
    }

    /**
     * The main repeating task that handles:
     * - Finding/validating target (only every TARGET_CHECK_INTERVAL ticks)
     * - Running pathfinding (only every PATHFIND_INTERVAL ticks)
     * - Following path or going direct if possible
     * - Spawning fewer particles (only every PARTICLE_INTERVAL ticks)
     */
    private void startOptimizedHomingTask(final Arrow arrow) {
        new BukkitRunnable() {
            private LivingEntity currentTarget;
            private LinkedList<Location> currentPath;
            private int ticksAlive = 0;

            @Override
            public void run() {
                ticksAlive += 1;

                // Cancel if arrow is invalid
                if (!isArrowInFlight(arrow)) {
                    cancel();
                    return;
                }

                // 1) Possibly spawn some minimal particles
                if (ticksAlive % PARTICLE_INTERVAL == 0) {
                    spawnParticleTrail(arrow);
                }

                // 2) Possibly find or re-check target
                if ((currentTarget == null || !isValidTarget(currentTarget))
                        && ticksAlive % TARGET_CHECK_INTERVAL == 0) {
                    currentTarget = findNearestEntityInCone(arrow, 30.0, 90.0);
                    currentPath = null; // reset path
                }

                // If no target, just fly normally
                if (currentTarget == null) {
                    return;
                }

                // 3) Attempt direct shot first
                if (attemptDirectShot(arrow, currentTarget)) {
                    return; // if direct shot is successful, skip path logic
                }

                // 4) If direct shot is blocked, maybe do pathfinding (but not every tick)
                if (currentPath == null && ticksAlive % PATHFIND_INTERVAL == 0) {
                    currentPath = computeAStarPath(arrow, currentTarget);
                }

                // 5) Follow existing path if we have one
                if (currentPath != null && !currentPath.isEmpty()) {
                    followPath(arrow, currentTarget, currentPath);
                }
                // else do nothing special this tick
            }
        }.runTaskTimer(HomingEnchantmentPlugin.getPlugin(HomingEnchantmentPlugin.class), 0L, 2L);
        // ^ main update still runs every 2 ticks, but heavy stuff is spaced out
    }

    // --------------------------------------------------------------------------------------------
    // Particles, Arrow Validation
    // --------------------------------------------------------------------------------------------

    private boolean isArrowInFlight(Arrow arrow) {
        return arrow != null
                && arrow.isValid()
                && !arrow.isDead()
                && !arrow.isInBlock()
                && !arrow.isOnGround();
    }

    private void spawnParticleTrail(Arrow arrow) {
        // A minimal amount of "END_ROD" for sparkle
        arrow.getWorld().spawnParticle(
                Particle.END_ROD,
                arrow.getLocation(),
                2, // count
                0.1, 0.1, 0.1,
                0.0);
        // A minimal amount of flame
        arrow.getWorld().spawnParticle(
                Particle.FLAME,
                arrow.getLocation(),
                4,
                0.1, 0.1, 0.1,
                0.0);
    }

    private boolean isValidTarget(LivingEntity e) {
        return (e != null) && e.isValid() && !e.isDead();
    }

    // --------------------------------------------------------------------------------------------
    // Direct Shots & Speed Boost
    // --------------------------------------------------------------------------------------------

    /**
     * If there's direct line-of-sight, steer arrow directly and possibly boost
     * speed.
     *
     * @return true if we successfully took a direct shot, false if line-of-sight is
     *         blocked.
     */
    private boolean attemptDirectShot(Arrow arrow, LivingEntity target) {
        Location arrowLoc = arrow.getLocation();
        Location targetMid = getTargetMidpoint(target);

        if (hasLineOfSight(arrow.getWorld(), arrowLoc, targetMid)) {
            steer(arrow, arrowLoc, targetMid);
            maybeBoostSpeed(arrow, arrowLoc, targetMid);
            return true;
        }
        return false;
    }

    /**
     * If the arrow is close (<= CLOSE_RANGE), multiply arrow velocity by
     * SPEED_BOOST_FACTOR.
     */
    private void maybeBoostSpeed(Arrow arrow, Location from, Location to) {
        if (from.distance(to) <= CLOSE_RANGE) {
            arrow.setVelocity(arrow.getVelocity().multiply(SPEED_BOOST_FACTOR));
        }
    }

    private void steer(Arrow arrow, Location from, Location to) {
        Vector dir = to.toVector().subtract(from.toVector()).normalize();
        double speed = arrow.getVelocity().length();
        arrow.setVelocity(dir.multiply(speed));
    }

    // --------------------------------------------------------------------------------------------
    // Following a Precomputed Path
    // --------------------------------------------------------------------------------------------

    private void followPath(Arrow arrow, LivingEntity target, LinkedList<Location> path) {
        Location arrowLoc = arrow.getLocation();
        if (path.isEmpty())
            return;

        // If we can see final target, let's skip the path
        Location finalNode = getTargetMidpoint(target);
        if (hasLineOfSight(arrow.getWorld(), arrowLoc, finalNode)) {
            steer(arrow, arrowLoc, finalNode);
            maybeBoostSpeed(arrow, arrowLoc, finalNode);
            path.clear(); // done with path
            return;
        }

        // Otherwise, aim for the next path node
        Location nextNode = path.getFirst();
        double distance = arrowLoc.distance(nextNode);

        // If we're close, pop it
        if (distance < 1.0) {
            path.removeFirst();
            if (path.isEmpty()) {
                return;
            }
            nextNode = path.getFirst();
        }

        steer(arrow, arrowLoc, nextNode);
    }

    // --------------------------------------------------------------------------------------------
    // Finding Nearest Target
    // --------------------------------------------------------------------------------------------

    private LivingEntity findNearestEntityInCone(Arrow arrow, double radius, double totalAngleDegrees) {
        ProjectileSource source = arrow.getShooter();
        if (!(source instanceof Entity)) {
            // Could be a dispenser or skeleton, skip if you only want real players
        }

        Vector arrowDir = arrow.getVelocity().normalize();
        double halfAngle = totalAngleDegrees / 2.0;
        double dotThreshold = Math.cos(Math.toRadians(halfAngle));

        double nearestDistSq = Double.MAX_VALUE;
        LivingEntity nearest = null;

        // This can still be expensive if many entities are in range
        // but it's much less than a big pathfinding loop
        List<Entity> nearby = arrow.getNearbyEntities(radius, radius, radius);
        for (Entity e : nearby) {
            if (!(e instanceof LivingEntity))
                continue;
            if (e == source)
                continue; // skip the shooter

            LivingEntity candidate = (LivingEntity) e;
            if (!isValidTarget(candidate))
                continue;

            // Check angle
            Vector toCandidate = candidate.getLocation().toVector()
                    .subtract(arrow.getLocation().toVector());
            double distSq = toCandidate.lengthSquared();
            // Quick skip if it's further than our radius squared
            if (distSq > radius * radius)
                continue;

            toCandidate.normalize();
            double dot = arrowDir.dot(toCandidate);
            if (dot < dotThreshold) {
                continue;
            }

            // (Optional) Check line-of-sight
            Location candidateMid = getTargetMidpoint(candidate);
            if (!hasLineOfSight(arrow.getWorld(), arrow.getLocation(), candidateMid)) {
                continue;
            }

            if (distSq < nearestDistSq) {
                nearestDistSq = distSq;
                nearest = candidate;
            }
        }
        return nearest;
    }

    private Location getTargetMidpoint(LivingEntity entity) {
        return entity.getLocation().add(0, entity.getHeight() * 0.5, 0);
    }

    // --------------------------------------------------------------------------------------------
    // A* Path Computation (Reduced + Capped)
    // --------------------------------------------------------------------------------------------

    /**
     * Runs a 3D block-based A* with a smaller radius and a limit on expansions.
     * If we exceed MAX_EXPANSIONS, we give up to avoid meltdown.
     */
    private LinkedList<Location> computeAStarPath(Arrow arrow, LivingEntity target) {
        // 1) If too far or no immediate need, skip
        Location arrowLoc = arrow.getLocation();
        Location targetLoc = target.getLocation();

        if (arrowLoc.distanceSquared(targetLoc) > SEARCH_RADIUS * SEARCH_RADIUS * 9) {
            return null;
        }

        // 2) Convert arrow's block + target's block
        Block start = arrowLoc.getBlock();
        Block end = targetLoc.getBlock();

        // 3) Actually run pathfinding
        return new AStarPathfinder(arrow.getWorld(), start, end, SEARCH_RADIUS).findPath();
    }

    // --------------------------------------------------------------------------------------------
    // Ray-based "Line of Sight"
    // --------------------------------------------------------------------------------------------

    private boolean hasLineOfSight(World world, Location start, Location end) {
        if (start == null || end == null)
            return false;

        Vector dir = end.toVector().subtract(start.toVector());
        double dist = dir.length();
        dir.normalize();

        // The 'rayTrace' is relatively cheap compared to a big path search,
        // but we still want to avoid spamming it too often
        RayTraceResult result = world.rayTrace(
                start,
                dir,
                dist,
                FluidCollisionMode.NEVER,
                true,
                0.1,
                entity -> false // skip entity collisions
        );

        // If null => no block was hit => line is clear
        return (result == null);
    }

    // --------------------------------------------------------------------------------------------
    // A* Classes (Optimized with expansions cap)
    // --------------------------------------------------------------------------------------------
    private static class AStarNode {
        final int x, y, z;
        double gCost = Double.MAX_VALUE;
        double hCost = 0;
        AStarNode parent;

        AStarNode(int x, int y, int z) {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        double fCost() {
            return gCost + hCost;
        }
    }

    private static class AStarPathfinder {
        private final World world;
        private final Block startBlock, endBlock;
        private final int maxRadius;
        private final Map<String, AStarNode> nodeCache = new HashMap<>();

        AStarPathfinder(World w, Block start, Block end, int radius) {
            this.world = w;
            this.startBlock = start;
            this.endBlock = end;
            this.maxRadius = radius;
        }

        LinkedList<Location> findPath() {
            // Basic checks
            int sx = startBlock.getX(), sy = startBlock.getY(), sz = startBlock.getZ();
            int ex = endBlock.getX(), ey = endBlock.getY(), ez = endBlock.getZ();

            AStarNode startNode = getOrCreateNode(sx, sy, sz);
            startNode.gCost = 0;
            startNode.hCost = heuristic(startNode, ex, ey, ez);

            PriorityQueue<AStarNode> openSet = new PriorityQueue<>(Comparator.comparingDouble(AStarNode::fCost));
            openSet.add(startNode);

            int expansions = 0; // track how many nodes we pop from openSet
            while (!openSet.isEmpty()) {
                AStarNode current = openSet.poll();
                expansions++;

                // If expansions exceed limit, bail out to avoid meltdown
                if (expansions > MAX_EXPANSIONS) {
                    return null;
                }

                // Goal check
                if (current.x == ex && current.y == ey && current.z == ez) {
                    // Build path
                    return buildPath(current);
                }

                for (AStarNode neighbor : getNeighbors(current, ex, ey, ez)) {
                    double newG = current.gCost + 1.0;
                    if (newG < neighbor.gCost) {
                        neighbor.gCost = newG;
                        neighbor.hCost = heuristic(neighbor, ex, ey, ez);
                        neighbor.parent = current;

                        if (!openSet.contains(neighbor)) {
                            openSet.add(neighbor);
                        }
                    }
                }
            }

            // No path found
            return null;
        }

        private LinkedList<Location> buildPath(AStarNode endNode) {
            LinkedList<Location> path = new LinkedList<>();
            AStarNode cur = endNode;
            while (cur != null) {
                path.addFirst(new Location(
                        world,
                        cur.x + 0.5,
                        cur.y + 0.5,
                        cur.z + 0.5));
                cur = cur.parent;
            }
            return path;
        }

        private double heuristic(AStarNode node, int ex, int ey, int ez) {
            double dx = (ex - node.x);
            double dy = (ey - node.y);
            double dz = (ez - node.z);
            return Math.sqrt(dx * dx + dy * dy + dz * dz);
        }

        private AStarNode getOrCreateNode(int x, int y, int z) {
            String key = x + "," + y + "," + z;
            return nodeCache.computeIfAbsent(key, k -> new AStarNode(x, y, z));
        }

        private List<AStarNode> getNeighbors(AStarNode node, int ex, int ey, int ez) {
            List<AStarNode> results = new ArrayList<>(6);

            // Up to 6 cardinal directions. You can skip vertical if you want simpler
            int[][] offsets = { { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 } };
            for (int[] off : offsets) {
                int nx = node.x + off[0];
                int ny = node.y + off[1];
                int nz = node.z + off[2];

                if (!inRange(nx, ny, nz))
                    continue;
                if (!isPassable(nx, ny, nz))
                    continue;

                results.add(getOrCreateNode(nx, ny, nz));
            }
            return results;
        }

        private boolean inRange(int x, int y, int z) {
            // limit expansions to a bounding box around startBlock
            return Math.abs(x - startBlock.getX()) <= maxRadius
                    && Math.abs(y - startBlock.getY()) <= maxRadius
                    && Math.abs(z - startBlock.getZ()) <= maxRadius;
        }

        private boolean isPassable(int x, int y, int z) {
            Material mat = world.getBlockAt(x, y, z).getType();
            // You might need more logic if you want the arrow to fit in a 2-block space
            return !mat.isSolid();
        }
    }
}

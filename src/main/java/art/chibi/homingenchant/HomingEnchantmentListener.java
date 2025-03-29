package art.chibi.homingenchant;

import org.bukkit.FluidCollisionMode;
import org.bukkit.GameMode;
import org.bukkit.Location;
import org.bukkit.Material;
import org.bukkit.Particle;
import org.bukkit.Particle.DustOptions;
import org.bukkit.util.RayTraceResult;
import org.bukkit.Registry;
import org.bukkit.World;
import org.bukkit.block.Block;
import org.bukkit.enchantments.Enchantment;
import org.bukkit.entity.Arrow;
import org.bukkit.entity.Entity;
import org.bukkit.entity.LivingEntity;
import org.bukkit.entity.Player;
import org.bukkit.event.EventHandler;
import org.bukkit.event.Listener;
import org.bukkit.event.entity.EntityShootBowEvent;
import org.bukkit.projectiles.ProjectileSource;
import org.bukkit.scheduler.BukkitRunnable;
import org.bukkit.util.Vector;

import io.papermc.paper.registry.RegistryAccess;
import io.papermc.paper.registry.RegistryKey;

import java.util.*;
// import java.util.logging.Logger;

public class HomingEnchantmentListener implements Listener {
    // private static final Logger LOGGER = Logger.getLogger("HomingEnchantment");
    // ------------------- CONFIG -------------------
    private static final int SEARCH_RADIUS = 80; // Smaller A* search radius
    private static final int MAX_EXPANSIONS = 800; // Max A* expansions before we bail
    private static final int PATHFIND_INTERVAL = 2; // Ticks between path computations
    private static final int TARGET_CHECK_INTERVAL = 2; // Ticks between target re-check
    private static final int PARTICLE_INTERVAL = 1; // Ticks between spawning trail dots

    // Particle lifetime constant: particles are re-spawned for 10 seconds = 200
    // ticks,
    // or until the arrow is no longer in flight.
    private static final int PARTICLE_DURATION_TICKS = 200;

    // New constants to control particle size growth.
    private static final float PARTICLE_INITIAL_SIZE = 1.0f;
    private static final float PARTICLE_GROWTH_RATE = 0.005f; // Increase per tick

    private static final double CLOSE_RANGE = 3.0; // Distance that triggers speed boost
    private static final double SPEED_BOOST_FACTOR = 3.0; // How much to speed up at close range
    // ------------------------------------------------

    // Map to store last known arrow positions for the dotted trail
    private final Map<UUID, Location> arrowLastLocations = new HashMap<>();

    @EventHandler
    public void onBowShoot(EntityShootBowEvent event) {
        if (!(event.getProjectile() instanceof Arrow))
            return;
        if (event.getBow() == null)
            return;

        RegistryAccess registry = RegistryAccess.registryAccess();
        Registry<Enchantment> enchantments = registry.getRegistry(RegistryKey.ENCHANTMENT);

        // Check if the bow has the custom "chibi:aimbot" enchant
        if (!event.getBow().containsEnchantment(enchantments.get(HomingEnchantmentConstants.ENCHANTMENT_KEY))) {
            return;
        }

        Arrow arrow = (Arrow) event.getProjectile();
        startOptimizedHomingTask(arrow);
    }

    /**
     * The main repeating task that handles:
     * - Finding/validating target (every TARGET_CHECK_INTERVAL ticks)
     * - Running pathfinding (every PATHFIND_INTERVAL ticks)
     * - Following path or direct shot if possible
     * - Spawning a persistent dotted trail (every PARTICLE_INTERVAL ticks)
     */
    private void startOptimizedHomingTask(final Arrow arrow) {
        new BukkitRunnable() {
            private LivingEntity currentTarget;
            private LinkedList<Location> currentPath;
            private int ticksAlive = 0;

            @Override
            public void run() {
                ticksAlive++;

                // Cancel if the arrow is no longer in flight.
                if (!isArrowInFlight(arrow)) {
                    arrowLastLocations.remove(arrow.getUniqueId());
                    cancel();
                    return;
                }

                // 1) Spawn a persistent dotted trail along the arrow's path.
                if (ticksAlive % PARTICLE_INTERVAL == 0) {
                    spawnDottedTrail(arrow);
                }

                // 2) (Re)search for a target if needed.
                if ((currentTarget == null || !isValidTarget(currentTarget))
                        && ticksAlive % TARGET_CHECK_INTERVAL == 0) {
                    currentTarget = findNearestEntityInCone(arrow, 30.0, 90.0);
                    currentPath = null; // reset path when target changes
                }

                // If no valid target, let the arrow fly normally.
                if (currentTarget == null) {
                    return;
                }

                // 3) Attempt a direct shot.
                if (attemptDirectShot(arrow, currentTarget)) {
                    return;
                }

                // 4) If direct shot is blocked, compute a path every PATHFIND_INTERVAL ticks.
                if (currentPath == null && ticksAlive % PATHFIND_INTERVAL == 0) {
                    currentPath = computeAStarPath(arrow, currentTarget);
                }

                // 5) Follow the computed path if available.
                if (currentPath != null && !currentPath.isEmpty()) {
                    followPath(arrow, currentTarget, currentPath);
                }
            }
        }.runTaskTimer(HomingEnchantmentPlugin.getPlugin(HomingEnchantmentPlugin.class), 0L, 2L);
    }

    // --------------------------------------------------------------------------------------------
    // Dotted Trail & Persistent Particle Scheduling
    // --------------------------------------------------------------------------------------------

    private boolean isArrowInFlight(Arrow arrow) {
        return arrow != null
                && arrow.isValid()
                && !arrow.isDead()
                && !arrow.isInBlock()
                && !arrow.isOnGround();
    }

    /**
     * Spawns a dotted trail along the arrow's flight path by interpolating between
     * its previous
     * and current positions. Each dot is scheduled to re-spawn persistently for a
     * fixed lifetime
     * (PARTICLE_DURATION_TICKS) or until the arrow is no longer in flight.
     */
    private void spawnDottedTrail(Arrow arrow) {
        Location currentLocation = arrow.getLocation();
        UUID arrowId = arrow.getUniqueId();
        Location previousLocation = arrowLastLocations.getOrDefault(arrowId, currentLocation);

        double distance = previousLocation.distance(currentLocation);
        double dotSpacing = 0.5;
        int dotCount = (int) (distance / dotSpacing);

        Vector direction = (distance > 0)
                ? currentLocation.toVector().subtract(previousLocation.toVector()).normalize()
                : new Vector(0, 0, 0);

        // For each dot along the path, schedule a persistent particle task.
        for (int i = 0; i <= dotCount; i++) {
            Location dotLocation = previousLocation.clone().add(direction.clone().multiply(i * dotSpacing));
            schedulePersistentParticle(arrow, dotLocation);
        }

        arrowLastLocations.put(arrowId, currentLocation);
    }

    /**
     * Schedules a task that re-spawns a dust particle at the given location every
     * tick.
     * The task cancels itself either when the arrow is no longer in flight or when
     * the particle has been re-spawned for PARTICLE_DURATION_TICKS ticks.
     * The particle size grows over time to simulate a rocket/missile trail.
     *
     * @param arrow    The arrow associated with this particle dot.
     * @param location The fixed location for the dot.
     */
    private void schedulePersistentParticle(final Arrow arrow, final Location location) {
        new BukkitRunnable() {
            int ticks = 0;

            @Override
            public void run() {
                // Cancel if the arrow is no longer in flight or lifetime exceeded.
                if (!isArrowInFlight(arrow) || ticks >= PARTICLE_DURATION_TICKS) {
                    cancel();
                    return;
                }
                // Compute current particle size based on lifetime progress.
                float currentSize = PARTICLE_INITIAL_SIZE + ticks * PARTICLE_GROWTH_RATE;
                DustOptions options = new DustOptions(org.bukkit.Color.WHITE, currentSize);
                location.getWorld().spawnParticle(
                        Particle.DUST,
                        location,
                        1,
                        0, 0, 0, // No offset; the dot appears exactly at the location.
                        options);
                ticks++;
            }
        }.runTaskTimer(HomingEnchantmentPlugin.getPlugin(HomingEnchantmentPlugin.class), 0L, 1L);
    }

    private boolean isValidTarget(LivingEntity e) {
        if (e == null || !e.isValid() || e.isDead() || e.getHealth() <= 0 || e.isInvulnerable())
            return false;
        if (e instanceof Player) {
            Player player = (Player) e;
            if (player.getGameMode() == GameMode.CREATIVE)
                return false;
        }
        return true;
    }

    // --------------------------------------------------------------------------------------------
    // Direct Shots & Speed Boost
    // --------------------------------------------------------------------------------------------

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
        // If the arrow has clear line-of-sight to the final target, use that.
        Location finalNode = getTargetMidpoint(target);
        if (hasLineOfSight(arrow.getWorld(), arrowLoc, finalNode)) {
            steer(arrow, arrowLoc, finalNode);
            maybeBoostSpeed(arrow, arrowLoc, finalNode);
            path.clear();
            return;
        }
        // Otherwise, aim for the next node in the path.
        Location nextNode = path.getFirst();
        double distance = arrowLoc.distance(nextNode);
        if (distance < 1.0) {
            path.removeFirst();
            if (path.isEmpty())
                return;
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
            // Skip if shooter is not a living entity.
        }
        Vector arrowDir = arrow.getVelocity().normalize();
        double halfAngle = totalAngleDegrees / 2.0;
        double dotThreshold = Math.cos(Math.toRadians(halfAngle));
        double nearestDistSq = Double.MAX_VALUE;
        LivingEntity nearest = null;
        List<Entity> nearby = arrow.getNearbyEntities(radius, radius, radius);
        for (Entity e : nearby) {
            if (!(e instanceof LivingEntity) || e == source)
                continue;
            LivingEntity candidate = (LivingEntity) e;
            if (!isValidTarget(candidate))
                continue;
            Vector toCandidate = candidate.getLocation().toVector().subtract(arrow.getLocation().toVector());
            double distSq = toCandidate.lengthSquared();
            if (distSq > radius * radius)
                continue;
            toCandidate.normalize();
            if (arrowDir.dot(toCandidate) < dotThreshold)
                continue;
            Location candidateMid = getTargetMidpoint(candidate);
            if (!hasLineOfSight(arrow.getWorld(), arrow.getLocation(), candidateMid))
                continue;
            if (distSq < nearestDistSq) {
                nearestDistSq = distSq;
                nearest = candidate;
            }
        }
        // if (nearest != null) {
        // LOGGER.info("Nearest: " + nearest.getName());
        // }
        return nearest;
    }

    private Location getTargetMidpoint(LivingEntity entity) {
        return entity.getLocation().add(0, entity.getHeight() * 0.5, 0);
    }

    // --------------------------------------------------------------------------------------------
    // A* Path Computation (Reduced + Capped)
    // --------------------------------------------------------------------------------------------

    private LinkedList<Location> computeAStarPath(Arrow arrow, LivingEntity target) {
        Location arrowLoc = arrow.getLocation();
        Location targetLoc = target.getLocation();
        if (arrowLoc.distanceSquared(targetLoc) > SEARCH_RADIUS * SEARCH_RADIUS * 9)
            return null;
        Block start = arrowLoc.getBlock();
        Block end = targetLoc.getBlock();
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
        RayTraceResult result = world.rayTrace(
                start,
                dir,
                dist,
                FluidCollisionMode.NEVER,
                true,
                0.1,
                entity -> false // Skip entity collisions.
        );
        return (result == null);
    }

    // --------------------------------------------------------------------------------------------
    // A* Classes (Optimized with Expansions Cap)
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
            int sx = startBlock.getX(), sy = startBlock.getY(), sz = startBlock.getZ();
            int ex = endBlock.getX(), ey = endBlock.getY(), ez = endBlock.getZ();

            AStarNode startNode = getOrCreateNode(sx, sy, sz);
            startNode.gCost = 0;
            startNode.hCost = heuristic(startNode, ex, ey, ez);

            PriorityQueue<AStarNode> openSet = new PriorityQueue<>(Comparator.comparingDouble(AStarNode::fCost));
            openSet.add(startNode);

            int expansions = 0;
            while (!openSet.isEmpty()) {
                AStarNode current = openSet.poll();
                expansions++;
                if (expansions > MAX_EXPANSIONS)
                    return null;
                if (current.x == ex && current.y == ey && current.z == ez)
                    return buildPath(current);
                for (AStarNode neighbor : getNeighbors(current, ex, ey, ez)) {
                    double newG = current.gCost + 1.0;
                    if (newG < neighbor.gCost) {
                        neighbor.gCost = newG;
                        neighbor.hCost = heuristic(neighbor, ex, ey, ez);
                        neighbor.parent = current;
                        if (!openSet.contains(neighbor))
                            openSet.add(neighbor);
                    }
                }
            }
            return null;
        }

        private LinkedList<Location> buildPath(AStarNode endNode) {
            LinkedList<Location> path = new LinkedList<>();
            AStarNode cur = endNode;
            while (cur != null) {
                path.addFirst(new Location(world, cur.x + 0.5, cur.y + 0.5, cur.z + 0.5));
                cur = cur.parent;
            }
            return path;
        }

        private double heuristic(AStarNode node, int ex, int ey, int ez) {
            double dx = ex - node.x;
            double dy = ey - node.y;
            double dz = ez - node.z;
            return Math.sqrt(dx * dx + dy * dy + dz * dz);
        }

        private AStarNode getOrCreateNode(int x, int y, int z) {
            String key = x + "," + y + "," + z;
            return nodeCache.computeIfAbsent(key, k -> new AStarNode(x, y, z));
        }

        private List<AStarNode> getNeighbors(AStarNode node, int ex, int ey, int ez) {
            List<AStarNode> results = new ArrayList<>(6);
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
            return Math.abs(x - startBlock.getX()) <= maxRadius
                    && Math.abs(y - startBlock.getY()) <= maxRadius
                    && Math.abs(z - startBlock.getZ()) <= maxRadius;
        }

        private boolean isPassable(int x, int y, int z) {
            Material mat = world.getBlockAt(x, y, z).getType();
            return !mat.isSolid();
        }
    }
}
